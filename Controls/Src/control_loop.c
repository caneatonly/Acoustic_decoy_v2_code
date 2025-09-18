#include "control_loop.h"
#include "depth_estimator.h"
#include "mission_manager.h"
#include "telemetry.h"
#include "sensor_process.h"
#include "MS5837_lib.h"
#include "control_config.h"
#include "depth_cascaded_ctrl.h"
#include "actuators.h"
#include "control_algorithm.h"
#include "balloon_state.h"
#include <stdint.h>
#include <stdbool.h>

static depth_estimator_t g_depth_est;
static uint32_t g_last_publish=0;
static depth_ctrl_t g_depth_ctrl = {0};
static balloon_status_t g_balloon = {0};

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
    DepthCtrl_Init(&g_depth_ctrl, &cfg, CTRL_DEPTH_TARGET_M);
}

// 控制循环初始化
void ControlLoop_Init(void){
    DepthEst_Init(&g_depth_est,ESTIMATOR_EMA_ALPHA_Z);
    Mission_Init(CTRL_DEPTH_TARGET_M); 
    DepthController_Init();
    Actuators_Init(); //空函数！！！
    Balloon_Init(&g_balloon);
    Telemetry_Init();  //空函数！！！
}

void ControlLoop_RunIteration(uint32_t now_ms){
    MS5837_Data_t *ms = MS5837_GetData();
    if(ms && ms->data_valid){
        DepthEst_Update(&g_depth_est, ms->depth, now_ms);
    }

    // 获取当前深度和速度估计
    float depth_est = DepthEst_GetDepth(&g_depth_est);
    float velocity_est = DepthEst_GetVelocity(&g_depth_est);

    // 由 Mission Manager 统一管理阶段切换：传入当前的气囊状态（使用上次更新的状态）
    Mission_Update(now_ms, depth_est, velocity_est, g_balloon.state);
    
    // 任务状态快照 用于上报
    const mission_status_t *mstat = Mission_GetStatus();
    Telemetry_SetMissionState((int)mstat->state);

    // 检测状态切换，并执行状态切换所需要的一次性动作
    bool state_changed = Mission_HasStateChanged();
    if (state_changed) {
        if (mstat->state == MISSION_PREP_HOLD) {
            // 进入预备阶段：释放整流罩（一次性动作），并启用气囊充气控制
            Actuators_FairingRelease();
            Valve_ControlAlgorithm_Enable(true);
        } else {
            // 其它任何阶段：确保充气控制关闭 ！！后续HOLD阶段可考虑开启微调
            Valve_ControlAlgorithm_Enable(false);
        }
        // 确认已处理状态切换的一次性动作
        Mission_AckStateChange();
    }

    // 根据状态机的状态配置控制器参数
    int16_t pwm;
    if (mstat->state == MISSION_APPROACH ||
        mstat->state == MISSION_PREP_HOLD ||
        mstat->state == MISSION_DEPTH_HOLD) {
        // 默认值
        depth_ctrl_mode_t mode = DEPTH_CTRL_MODE_APPROACH;
        bool force_vref = false;
        float vref_cmd = 0.0f;

        switch (mstat->state) {
            case MISSION_APPROACH:
                mode = DEPTH_CTRL_MODE_APPROACH;
                force_vref = false; // 外环产生速度参考
                break;
            case MISSION_PREP_HOLD:
                mode = DEPTH_CTRL_MODE_HOLD;     // 进入预备带后改为保深增益
                force_vref = true;               // 强制 v_ref = 0，速度趋近0
                vref_cmd = 0.0f;
                break;
            case MISSION_DEPTH_HOLD:
                mode = DEPTH_CTRL_MODE_HOLD;
                force_vref = false; // 由外环给出较小速度参考
                break;
            default: break;
        }
        DepthCtrl_SetMode(&g_depth_ctrl, mode);
        DepthCtrl_ForceVref(&g_depth_ctrl, force_vref, vref_cmd);
        DepthCtrl_Update(&g_depth_ctrl, depth_est, velocity_est, now_ms);
        pwm = DepthCtrl_GetPwm(&g_depth_ctrl);
    } else {
        // 其它状态不驱动电机（可后续细化不同状态策略）
        pwm = CTRL_PWM_NEUTRAL;
    }

    Actuators_SetMotorPwm(pwm);
    
    // 电磁阀充气算法执行，更新气囊状态
    Valve_ControlAlgorithm_Update();
    float duty = Valve_GetDuty();
    float p_bag = Valve_GetPbag();
    float p_water = Valve_GetPwater();
    float dPdt = Valve_GetdPdt();
    Balloon_Update(&g_balloon, duty, p_bag, p_water, dPdt, now_ms);


    Telemetry_SetBalloon((int)g_balloon.state);
    // 状态数据更新与发布
    Telemetry_SetDepth(depth_est, velocity_est);
    float vref_pub = (mstat->state == MISSION_APPROACH || mstat->state == MISSION_PREP_HOLD || mstat->state == MISSION_DEPTH_HOLD)
                        ? DepthCtrl_GetVref(&g_depth_ctrl) : 0.0f;
    Telemetry_SetControl(vref_pub, pwm);
    Telemetry_SetPressures(p_bag, p_water, dPdt, duty);
    if(now_ms - g_last_publish >= CTRL_STATUS_PUBLISH_MS){
        Telemetry_Publish(now_ms);
        g_last_publish = now_ms;
    }
}
