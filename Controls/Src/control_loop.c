#include "control_loop.h"
#include "depth_estimator.h"
#include "mission_manager.h"
#include "telemetry.h"
#include "sensors_data_get.h"
#include "control_config.h"
#include "depth_cascaded_ctrl.h"
#include "actuators.h"
#include "mission_exec.h"
#include "valve_ctrl.h"
#include "balloon_state.h"
#include <stdint.h>
#include <stdbool.h>

// 装置总体任务状态由 Mission Manager 管理，任务具体执行由Mission exec执行

// 全局静态变量
// 设备总状态结构体全局变量在mission_manager.c中定义


/* 1） 深度估计器全局结构体：
        记录当前： 未滤波深度，滤波深度，速度估计，EMA系数，时间戳，有效性标志*/
static depth_estimator_t g_depth_est;
/* 2） 深度控制器全局结构体：
        记录当前： PID增益，限制参数，控制模式，目标深度，速度参考，PWM输出*/
static depth_ctrl_t g_depth_ctrl = {0};
/* 3） 气囊状态机全局结构体：
        记录当前： 气囊状态*/
static balloon_status_t g_balloon = {0};

// 上次状态发布的时间戳，用于无阻塞发布消息
static uint32_t g_last_publish = 0;

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
    cfg.dir_thresh_pwm = CTRL_PWM_DIR_THRESH;
    // Initialize with current mission target depth
    const mission_status_t* ms = Mission_GetStatus();
    float target = ms ? ms->target_depth_m : CTRL_DEPTH_TARGET_M;
    DepthCtrl_Init(&g_depth_ctrl, &cfg, target);
}

// 控制循环初始化
void ControlLoop_Init(void){
    DepthEst_Init(&g_depth_est,ESTIMATOR_EMA_ALPHA_Z, ESTIMATOR_VEL_BETA);
    Mission_Init(CTRL_DEPTH_TARGET_M);
    DepthController_Init();
    Balloon_Init(&g_balloon);
    Telemetry_Init();  //空函数！！！
}

/*!!!总体控制逻辑大循环!!!

    1.深度估计器更新
    2.Mission Manager 统一管理阶段切换
    3.Mission Execute 根据任务状态执行动作
    4.气囊状态机更新
    5.状态数据更新与发布
*/
void ControlLoop_RunIteration(uint32_t now_ms){

    // 深度估计器更新
    const MS5837_Data_t *ms = MS5837_data_get();
    if(ms && ms->data_valid){
        DepthEst_Update(&g_depth_est, ms->depth, now_ms);
    }

    // 获取当前深度和速度估计
    float depth_est = DepthEst_GetDepth(&g_depth_est);
    float velocity_est = DepthEst_GetVelocity(&g_depth_est);

    // Mission Manager 统一管理阶段切换 
    // 更新任务状态
    Mission_Update(now_ms, depth_est, velocity_est, g_balloon.state);
    
    // 获取当前任务状态
    mission_status_t *mstat = Mission_GetStatus();
    Telemetry_SetMissionState((int)mstat->state);

    // 根据任务状态执行动作
    Mission_Execute(now_ms, depth_est, velocity_est, &g_depth_ctrl, mstat);

    float duty = Valve_GetDuty();
    float p_bag = Valve_GetPbag();
    float p_water = Valve_GetPwater();
    float dPdt = Valve_GetdPdt();
    // 气囊状态机更新
    Balloon_Update(&g_balloon, duty, dPdt, now_ms);

    // 状态数据更新与发布
    Telemetry_SetDepth(depth_est, velocity_est, mstat->target_depth_m);
    Telemetry_SetBalloon((int)g_balloon.state);
    float vref_pub = (mstat->state == MISSION_APPROACH || mstat->state == MISSION_PREP_HOLD || mstat->state == MISSION_DEPTH_HOLD)
                        ? DepthCtrl_GetVref(&g_depth_ctrl) : 0.0f;
    int16_t pwm_pub = mstat->motor_active ? DepthCtrl_GetPwm(&g_depth_ctrl) : CTRL_PWM_NEUTRAL;
    Telemetry_SetControl(vref_pub, pwm_pub);

    Telemetry_SetPressures(p_bag, p_water, dPdt, duty);
    if(now_ms - g_last_publish >= CTRL_STATUS_PUBLISH_MS){
        Telemetry_Publish(now_ms);
        g_last_publish = now_ms;
    }
}
