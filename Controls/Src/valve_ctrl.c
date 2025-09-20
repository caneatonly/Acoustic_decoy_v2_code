// PD-based valve control with 1s duty window
#include "valve_ctrl.h"
#include "actuators.h"
#include "sensors_data_get.h"
#include "stm32f1xx_hal.h"

#include <math.h>

#define Twin_ms 1000    // 1s window

static float duty; 
// PD Controller Parameters 
static float  Kp_valve = 0.015f;           
static float  Kd_valve = 0.30f;           //待调整
static float  eps = 2.0f;           // 死区，<2.0Kpa（容忍20cm水深差距）
static float  dp_margin = 1.0f;     // 盈余压力，目标压力高于水压的部分,kPa 

// EMA and derivative filters
static float  alpha_bag = 0.20f;    // 0.1~0.3
static float  alpha_water = 0.20f;  // 0.1~0.3
static float  beta_d = 0.30f;       // 0.2~0.4 for low-noise derivative

// Window timing (ms)

static uint32_t  Tmin_on = 100;     // ms
static uint32_t  Tmin_off = 100;    // ms

// Safety
static float  guard_over_kpa = 30.0f;   // !!气囊压力永远不能大于 p_water + 30kPa

// 状态变量
static bool initialized = false;
static bool enabled = false;      // 是否启用充气功能，由状态机赋值
static uint32_t  last_update_ms = 0;
static uint32_t  win_start_ms = 0;
static uint32_t  locked_on_ms = 0;   // on-time locked at window start

static float  P_bag_filt = 0.0f;
static float  P_water_filt = 0.0f;
static float  dP_bag_dt = 0.0f;      // filtered derivative kPa/s
static float  P_bag_prev = 0.0f;

// Local non-blocking valve pulse job (moved from peripherals layer)
typedef struct {
    uint8_t active;
    uint32_t t_start_ms;
    uint32_t duration_ms;
} ValvePulseJob_t;
static ValvePulseJob_t g_valve_job = {0};

// Clamp helper
static inline float clampf(float x, float a, float b) { return (x < a) ? a : (x > b) ? b : x; }

void Valve_ControlAlgorithm_Init(void)
{
    uint32_t now = HAL_GetTick();
    last_update_ms = now;
    win_start_ms = now;
    locked_on_ms = 0;

    // Seed filters from current readings if available
    const BaroADC_Data_t* baro = Ballon_pressure_get();
    const MS5837_Data_t* ms5837 = MS5837_data_get();

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
    g_valve_job.active = 0;
    Actuators_ValveClose();
}

void Valve_ControlAlgorithm_Update(void)
{
    // Gate: when disabled, keep valve closed and progress job state only
    if (!enabled) {
        Actuators_ValveClose();
        valve_pulse_task();
        return;
    }

    // Initialize only when enabled to seed filters with up-to-date readings
    if (!initialized) {
        Valve_ControlAlgorithm_Init();
        initialized = true;
    }

    uint32_t now = HAL_GetTick();
    float dt = (now -  last_update_ms) * 0.001f; // seconds
    if (dt <= 0.0f || dt > 0.5f) dt = 0.1f;       // guard dt
    last_update_ms = now;

    // Read sensors
    const BaroADC_Data_t* baro = Ballon_pressure_get();
    const MS5837_Data_t* ms = MS5837_data_get();

    if (!baro || !ms || !baro->data_valid || !ms->data_valid) {
        // Invalidate control; ensure valve closed and progress job state machine
        Actuators_ValveClose();
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
    duty = (fabsf(e) <  eps) ? 0.0f : ( Kp_valve * e -  Kd_valve *  dP_bag_dt);
    duty = clampf(duty, 0.0f, 1.0f);

    // Safety: hard overpressure guard
    if ( P_bag_filt >=  P_water_filt +  guard_over_kpa) {
        duty = 0.0f;
        Actuators_ValveClose();
        // Also cancel any running job
    g_valve_job.active = 0;
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
            Actuators_ValveClose();
        }
    }

    // Progress the non-blocking pulse job
    valve_pulse_task();
}

void Valve_ControlAlgorithm_Enable(bool en)
{
    enabled = en;
    if (!enabled) {
        // Immediately ensure valve is closed, cancel any pending job, and mark uninitialized
        Actuators_ValveClose();
        g_valve_job.active = 0;
        initialized = false;
    } else {
        // On enable, re-initialize to capture current sensor baseline
        Valve_ControlAlgorithm_Init();
        initialized = true;
    }
}

bool Valve_ControlAlgorithm_IsEnabled(void)
{
    return enabled;
}

// Optional: expose simple setters for tuning (can be extended later)
void Valve_ControlAlgorithm_SetGains(float Kp_new, float Kd_new)
{
    if (Kp_new >= 0.0f) Kp_valve = Kp_new;
    if (Kd_new >= 0.0f) Kd_valve = Kd_new;
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
    out->Kp = Kp_valve;
    out->Kd = Kd_valve;
    out->eps_kpa = eps;
    out->dp_margin_kpa = dp_margin;
    out->guard_over_kpa = guard_over_kpa;
    out->window_ms = Twin_ms;
    out->Tmin_on_ms = Tmin_on;
    out->Tmin_off_ms = Tmin_off;
}

// Lightweight getters to expose telemetry for balloon-state and logging
float Valve_GetDuty(void) { return duty; }
float Valve_GetPbag(void) { return P_bag_filt; }
float Valve_GetPwater(void) { return P_water_filt; }
float Valve_GetdPdt(void) { return dP_bag_dt; }

// Non-blocking pulse API ownership moved here
void valve_open_for(uint32_t ms)
{
    if (ms == 0) return;
    uint32_t now = HAL_GetTick();
    if (!g_valve_job.active) {
    Actuators_ValveOpen();
        g_valve_job.active = 1;
        g_valve_job.t_start_ms = now;
        g_valve_job.duration_ms = ms;
    }
}

void valve_pulse_task(void)
{
    if (!g_valve_job.active) return;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - g_valve_job.t_start_ms) >= g_valve_job.duration_ms) {
    Actuators_ValveClose();
        g_valve_job.active = 0;
    }
}
