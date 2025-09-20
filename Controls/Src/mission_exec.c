#include "mission_exec.h"
#include "actuators.h"
#include "valve_ctrl.h"
#include "control_config.h"

void Mission_Execute(uint32_t now_ms, float depth_est, float velocity_est, depth_ctrl_t* ctrl, mission_status_t* s)
{
    if (!s || !ctrl) return;

    // 构建当期周期的动作意图
    const bool state_changed = Mission_HasStateChanged();

    // Defaults per cycle
    s->fairing_release_once = false;
    s->valve_enable = false;
    s->motor_active = false;
    s->ctrl_mode = DEPTH_CTRL_MODE_APPROACH;
    s->force_vref = false;
    s->vref_cmd = 0.0f;

    // 阶段资源使能策略：
    // - APPROACH:    motor=ON,  valve=OFF
    // - PREP_HOLD:   motor=ON,  valve=ON (释放罩+充气，强制v=0)
    // - DEPTH_HOLD:  motor=ON,  valve=ON（维持dp裕量）
    // - DWELL:       motor=OFF, valve=ON（允许小幅浮力微调）
    // - RECOVERY:    motor=ON,  valve=OFF（主动上浮）
    switch (s->state) {
        case MISSION_APPROACH:
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_APPROACH;
            s->force_vref = false;
            s->valve_enable = false;
            break;
        case MISSION_PREP_HOLD:
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_HOLD;
            s->force_vref = true;
            s->vref_cmd = 0.0f;
            s->valve_enable = true;
            if (state_changed) {
                s->fairing_release_once = true;
                // 进入零速保持前重置控制器积分，避免残余积分导致漂移
                // Reset controller integrators before entering zero-speed hold to avoid residual drift
                DepthCtrl_ResetIntegrators(ctrl);
            }
            break;
        case MISSION_DEPTH_HOLD:
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_HOLD;
            s->force_vref = false; 
            s->valve_enable = true; 
            break;
        case MISSION_DWELL_MONITOR:
            s->motor_active = false; 
            s->valve_enable = true;  
            s->ctrl_mode = DEPTH_CTRL_MODE_HOLD;
            s->force_vref = true;
            s->vref_cmd = 0.0f;
            break;
        case MISSION_RECOVERY_ASCEND:
            // 回收阶段：打开电机，命令上浮速度
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_APPROACH; // 用Approach的速度上限
            s->force_vref = true;
            s->vref_cmd = CTRL_RECOVERY_ASCEND_VREF;
            s->valve_enable = false; // 显式：回收阶段阀门失能
            if (state_changed) {
                // Before ascending, turn on 12V peripheral power
                Actuators_12V_PowerOn();
                DepthCtrl_ResetIntegrators(ctrl);
            }
            break;
        default:
            // keep defaults
            break;
    }

    // 执行一次性动作与持续控制（原 ExecTick）
    if (s->fairing_release_once) {
        Actuators_FairingRelease();
        Mission_AckStateChange();
    }

    // 阀门控制：仅在状态变化时切换
    if (Valve_ControlAlgorithm_IsEnabled() != s->valve_enable) {
        Valve_ControlAlgorithm_Enable(s->valve_enable);
    }

    // 深度控制与电机输出
    int16_t pwm = CTRL_PWM_NEUTRAL;
    if (s->motor_active) {
        DepthCtrl_SetMode(ctrl, s->ctrl_mode);
        DepthCtrl_ForceVref(ctrl, s->force_vref, s->vref_cmd);
        // 深度控制器更新（核心实现）
        DepthCtrl_Update(ctrl, depth_est, velocity_est, now_ms);
        pwm = DepthCtrl_GetPwm(ctrl);
    }
    
    // 推进器输出
    Actuators_SetMotorPwm(pwm);

    // 气阀控制（核心实现）
    Valve_ControlAlgorithm_Update();

}
