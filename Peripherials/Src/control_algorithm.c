// PD-based valve control with 1s duty window
#include "control_algorithm.h"

#include "sensor_process.h"   
#include "baro_adc.h"         

#include <math.h>

#define Twin_ms 1000    // 1s window

// PD Controller Parameters 
static float  Kp = 0.04f;           
static float  Kd = 0.30f;           
static float  eps = 1.5f;           // 死区，<1.5kPa
static float  dp_margin = 5.0f;     // 盈余压力，目标压力高于水压的部分，kPa

// EMA and derivative filters
static float  alpha_bag = 0.20f;    // 0.1~0.3
static float  alpha_water = 0.20f;  // 0.1~0.3
static float  beta_d = 0.30f;       // 0.2~0.4 for low-noise derivative

// Window timing (ms)

static uint32_t  Tmin_on = 100;     // ms
static uint32_t  Tmin_off = 200;    // ms

// Safety
static float  guard_over_kpa = 30.0f;   // P_bag should never exceed P_water + this

// 状态变量
static bool initialized = false;
static uint32_t  last_update_ms = 0;
static uint32_t  win_start_ms = 0;
static uint32_t  locked_on_ms = 0;   // on-time locked at window start

static float  P_bag_filt = 0.0f;
static float  P_water_filt = 0.0f;
static float  dP_bag_dt = 0.0f;      // filtered derivative kPa/s
static float  P_bag_prev = 0.0f;

// Clamp helper
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }

void Valve_ControlAlgorithm_Init(void)
{
	uint32_t now = HAL_GetTick();
	last_update_ms = now;
	win_start_ms = now;
	locked_on_ms = 0;

	// Seed filters from current readings if available
	const BaroADC_Data_t* baro = BaroADC_GetData();
	const MS5837_Data_t* ms5837 = MS5837_GetData();

	float P_bag = baro ? baro->pressure_bag : 0.0f;
	float P_water = 0.0f;
	if (ms5837 && ms5837->pressure_water > 0.0f) {
		P_water = ms5837->pressure_water;
	} 
	P_bag_filt = P_bag;
	P_water_filt = P_water;
	P_bag_prev = P_bag;
	dP_bag_dt = 0.0f;

	// Ensure valve job is idle
	ValvePulseJob_t* job = Valve_GetData();
	if (job) {
		job->active = false;
	}
	valve_close();
}

void Valve_ControlAlgorithm_Update(void)
{
    if (!initialized) {
        Valve_ControlAlgorithm_Init();
        initialized = true;
    }

	uint32_t now = HAL_GetTick();
	float dt = (now -  last_update_ms) * 0.001f; // seconds
	if (dt <= 0.0f || dt > 0.5f) dt = 0.1f;       // guard dt
	last_update_ms = now;

	// Read sensors
	const BaroADC_Data_t* baro = BaroADC_GetData();
	const MS5837_Data_t* ms = MS5837_GetData();

	if (!baro || !ms || !baro->data_valid || !ms->data_valid) {
		// Invalidate control; ensure valve closed and progress job state machine
		valve_close();
		valve_pulse_task();
		return;
	}

	float P_bag_raw = baro->pressure_bag; // kPa
	float P_water_raw = ms->pressure_water;

	// EMA filters
	 P_bag_filt   +=  alpha_bag   * (P_bag_raw   -  P_bag_filt);
	 P_water_filt +=  alpha_water * (P_water_raw -  P_water_filt);

	// Low-noise derivative on measurement
	float d_raw = ( P_bag_filt -  P_bag_prev) / dt; // kPa/s
	P_bag_prev =  P_bag_filt;
	dP_bag_dt += beta_d * (d_raw - dP_bag_dt);

	// Target and error
	float P_target =  P_water_filt +  dp_margin;   // kPa
	float e = P_target -  P_bag_filt;               // kPa

	// PD control -> duty in [0,1]
	float duty = (fabsf(e) <  eps) ? 0.0f : ( Kp * e -  Kd *  dP_bag_dt);
	duty = clampf(duty, 0.0f, 1.0f);

	// Safety: hard overpressure guard
	if ( P_bag_filt >=  P_water_filt +  guard_over_kpa) {
		duty = 0.0f;
		valve_close();
		// Also cancel any running job
		ValvePulseJob_t* job = Valve_GetData();
		if (job) job->active = false;
	}

	// Window scheduler: lock on-time at beginning of each 1s window
	uint32_t win_elapsed = now -  win_start_ms;
	if (win_elapsed >=  Twin_ms) {
		// Start a new window
		 win_start_ms = now;

		// Compute on-time for this window (respect min on/off)
		// Only apply Tmin_on when duty > 0, otherwise keep valve closed for the full window.
		uint32_t on_ms = 0;
		if (duty > 0.0f) {
			on_ms = (uint32_t)(duty * (float)Twin_ms + 0.5f);
			if (on_ms < Tmin_on) {
				on_ms = Tmin_on;
			}
			if (on_ms > Twin_ms - Tmin_off) {
				on_ms = Twin_ms - Tmin_off;
			}
		}

		 locked_on_ms = on_ms;

		// Fire one pulse for this window (non-blocking)
		if ( locked_on_ms > 0) {
			valve_open_for( locked_on_ms);
		} else {
			valve_close();
		}
	}

	// Progress the non-blocking pulse job
	valve_pulse_task();
}

// Optional: expose simple setters for tuning (can be extended later)
void Valve_ControlAlgorithm_SetGains(float Kp_new, float Kd_new)
{
	if (Kp_new >= 0.0f) Kp = Kp_new;
	if (Kd_new >= 0.0f) Kd = Kd_new;
}

void Valve_ControlAlgorithm_SetMargin(float dp_margin_kpa)
{
	if (dp_margin_kpa >= 0.0f)  dp_margin = dp_margin_kpa;
}

void Valve_ControlAlgorithm_SetWindow(uint32_t min_on_ms, uint32_t min_off_ms)
{
	// 简单约束：不允许最小开+最小关超过窗口
	if (min_on_ms > Twin_ms) min_on_ms = Twin_ms;
	if (min_off_ms > Twin_ms) min_off_ms = Twin_ms;
	if (min_on_ms + min_off_ms > Twin_ms) {
		// 优先保证最小关
		if (min_off_ms < Twin_ms) {
			min_on_ms = Twin_ms - min_off_ms;
		} else {
			min_on_ms = 0;
			min_off_ms = Twin_ms;
		}
	}
	Tmin_on = min_on_ms;
	Tmin_off = min_off_ms;
}

void Valve_ControlAlgorithm_SetEps(float eps_kpa)
{
	if (eps_kpa >= 0.0f) eps = eps_kpa;
}

void Valve_ControlAlgorithm_SetGuard(float over_kpa)
{
	if (over_kpa >= 0.0f) guard_over_kpa = over_kpa;
}

void Valve_ControlAlgorithm_GetParams(ValveControlParams_t* out)
{
	if (!out) return;
	out->Kp = Kp;
	out->Kd = Kd;
	out->eps_kpa = eps;
	out->dp_margin_kpa = dp_margin;
	out->guard_over_kpa = guard_over_kpa;
	out->window_ms = Twin_ms;
	out->Tmin_on_ms = Tmin_on;
	out->Tmin_off_ms = Tmin_off;
}

