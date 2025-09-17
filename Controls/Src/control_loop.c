#include "control_loop.h"
#include "depth_estimator.h"
#include "mission_manager.h"
#include "telemetry.h"
#include "sensor_process.h"
#include "MS5837_lib.h"
#include "control_config.h"
#include "depth_cascaded_ctrl.h"
#include "actuators.h"
#include <stdint.h>
#include <stdbool.h>

static depth_estimator_t g_depth_est;
static uint32_t g_last_publish=0;
static depth_ctrl_t g_depth_ctrl = {0};

/*  控制器初始化函数
    1.读取PID增益和限制参数
    2.初始化mode为APPROACH
    3.设置初始目标深度
 */
static void DepthController_Init(void){
    depth_ctrl_config_t cfg = {0};
    cfg.depth_app.kp = PID_Z_KP_APP; cfg.depth_app.ki = PID_Z_KI_APP; cfg.depth_app.kd = PID_Z_KD_APP;
    cfg.vel_app.kp   = PID_V_KP_APP; cfg.vel_app.ki   = PID_V_KI_APP; cfg.vel_app.kd   = PID_V_KD_APP;
    cfg.depth_hold.kp= PID_Z_KP_HOLD; cfg.depth_hold.ki= PID_Z_KI_HOLD; cfg.depth_hold.kd= PID_Z_KD_HOLD;
    cfg.vel_hold.kp  = PID_V_KP_HOLD; cfg.vel_hold.ki  = PID_V_KI_HOLD; cfg.vel_hold.kd  = PID_V_KD_HOLD;
    cfg.v_ref_max_app = CTRL_V_REF_MAX_APP; cfg.v_ref_max_hold = CTRL_V_REF_MAX_HOLD; cfg.v_ref_slew = CTRL_V_REF_SLEW;
    cfg.pwm_neutral = CTRL_PWM_NEUTRAL; cfg.pwm_min = CTRL_PWM_MIN; cfg.pwm_max = CTRL_PWM_MAX; cfg.pwm_slew_per_tick = CTRL_PWM_SLEW_PER_TICK;
    DepthCtrl_Init(&g_depth_ctrl, &cfg, CTRL_PREP_DEPTH_TARGET_M);
}

// 控制循环初始化
void ControlLoop_Init(void){
    DepthEst_Init(&g_depth_est,ESTIMATOR_EMA_ALPHA_Z);
    Mission_Init(CTRL_PREP_DEPTH_TARGET_M); 
    DepthController_Init();
    Actuators_Init();//没看这个
}

void ControlLoop_RunIteration(uint32_t now_ms){
    MS5837_Data_t *ms = MS5837_GetData();
    if(ms && ms->data_valid){
        DepthEst_Update(&g_depth_est, ms->depth, now_ms);
    }

    float depth_est = DepthEst_GetDepth(&g_depth_est);
    float velocity_est = DepthEst_GetVelocity(&g_depth_est);
    Mission_Update(now_ms, depth_est, velocity_est);

    // 仅在任务状态进入 MISSION_APPROACH 后才启动深度控制器输出，之前保持中立 PWM
    const mission_status_t *mstat = Mission_GetStatus();
    int16_t pwm;
    if(mstat->state == MISSION_APPROACH ){
        DepthCtrl_Update(&g_depth_ctrl, depth_est, velocity_est, now_ms);
        pwm = DepthCtrl_GetPwm(&g_depth_ctrl);
    } else {
        pwm = CTRL_PWM_NEUTRAL; // 其它状态不驱动电机（可后续细化不同状态策略）
    }
    Actuators_SetMotorPwm(pwm);
    // 状态数据更新与发布
    Telemetry_SetDepth(depth_est, velocity_est);
    float vref_pub = (mstat->state == MISSION_APPROACH) ? DepthCtrl_GetVref(&g_depth_ctrl) : 0.0f;
    Telemetry_SetControl(vref_pub, pwm);
    if(now_ms - g_last_publish >= CTRL_STATUS_PUBLISH_MS){
        Telemetry_Publish(now_ms);
        g_last_publish = now_ms;
    }
}
