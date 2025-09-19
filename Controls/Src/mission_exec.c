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

    switch (s->state) {
        case MISSION_APPROACH:
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_APPROACH;
            s->force_vref = false;
            break;
        case MISSION_PREP_HOLD:
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_HOLD;
            s->force_vref = true;
            s->vref_cmd = 0.0f;
            s->valve_enable = true;
            if (state_changed) {
                s->fairing_release_once = true;
            }
            break;
        case MISSION_DEPTH_HOLD:
            s->motor_active = true;
            s->ctrl_mode = DEPTH_CTRL_MODE_HOLD;
            s->force_vref = false;
            break;
        default:
            // keep defaults
            break;
    }

    // 2) 执行一次性动作与持续控制（原 ExecTick）
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
