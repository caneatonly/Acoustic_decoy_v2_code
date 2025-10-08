#include "depth_cascaded_ctrl.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

static float clampf(float x,float a,float b){return x<a?a:(x>b?b:x);} 

// 初始化控制器
void DepthCtrl_Init(depth_ctrl_t *ctrl, const depth_ctrl_config_t *cfg, float z_target_init){
    if(!ctrl||!cfg) return; 
    ctrl->cfg = *cfg;
    ctrl->mode = DEPTH_CTRL_MODE_APPROACH; 
    ctrl->z_target = z_target_init; 
    ctrl->pwm_cmd = ctrl->cfg.pwm_neutral; 
    ctrl->v_ref = 0.0f;
    ctrl->v_ref_prev = 0.0f;
    ctrl->integ_z = 0.0f;
    ctrl->integ_v = 0.0f;
    ctrl->last_dir = 0;
    ctrl->force_vref_active = false;
    ctrl->force_vref_value = 0.0f;
    ctrl->initialized = true; 
}

// 设置深度控制模式
void DepthCtrl_SetMode(depth_ctrl_t *ctrl, depth_ctrl_mode_t mode){ 
    if(!ctrl) return; 
    ctrl->mode = mode; 
}

// 设置目标深度
void DepthCtrl_SetTarget(depth_ctrl_t *ctrl, float z_target){ 
    if(!ctrl) return; 
    ctrl->z_target = z_target; 
}

void DepthCtrl_ForceVref(depth_ctrl_t *ctrl, bool enable, float v_ref_fixed){
    if(!ctrl) return;
    ctrl->force_vref_active = enable;
    if(enable){
        ctrl->force_vref_value = v_ref_fixed;
        // also set current references to avoid jumps
        ctrl->v_ref = v_ref_fixed;
        ctrl->v_ref_prev = v_ref_fixed;
    }
}

// 控制器更新函数
void DepthCtrl_Update(depth_ctrl_t *ctrl, float z_meas, float v_meas, uint32_t now_ms){
    
    if(!ctrl||!ctrl->initialized) {return;}
    (void)now_ms;

    const pid_gains_t *gz; 
    const pid_gains_t *gv; 
    float v_ref_max;
    // Select gains based on mode
    if(ctrl->mode == DEPTH_CTRL_MODE_APPROACH){
        gz = &ctrl->cfg.depth_app; 
        gv = &ctrl->cfg.vel_app; 
        v_ref_max = ctrl->cfg.v_ref_max_app;
    } else {
        gz = &ctrl->cfg.depth_hold; 
        gv = &ctrl->cfg.vel_hold; 
        v_ref_max = ctrl->cfg.v_ref_max_hold;
    }

    // dt estimation (optional future use)


    // Depth error
    float ez = ctrl->z_target - z_meas;

    // 外环：深度环 (only PI now; D optional later)
    ctrl->integ_z += gz->ki * ez; // basic integration; anti-windup to be refined
    // 积分限幅
    ctrl->integ_z = clampf(ctrl->integ_z, -v_ref_max, v_ref_max);
    float v_ref_cmd = gz->kp * ez + ctrl->integ_z; // ignoring kd for now
    // 在强制速度参考时覆盖外环输出（例如PREP_HOLD阶段要求速度=0）
    if (ctrl->force_vref_active){
        v_ref_cmd = ctrl->force_vref_value;
    }
    // 内环速度参考限幅与斜率限制
    v_ref_cmd = clampf(v_ref_cmd, -v_ref_max, v_ref_max);
        float dv = v_ref_cmd - ctrl->v_ref_prev;
        float dv_lim;
        if (dv > ctrl->cfg.v_ref_slew) {
            dv_lim = ctrl->cfg.v_ref_slew;
        } else if (dv < -ctrl->cfg.v_ref_slew) {
            dv_lim = -ctrl->cfg.v_ref_slew;
        } else {
            dv_lim = dv;
        }

    ctrl->v_ref = ctrl->v_ref_prev + dv_lim;
    ctrl->v_ref_prev = ctrl->v_ref;

    //内环：速度环 (PI basic)
    const float pwm_span = (float)(ctrl->cfg.pwm_max - ctrl->cfg.pwm_min);
    const float guard = 0.05f * pwm_span;
    const float pwm_high_guard = (float)ctrl->cfg.pwm_max - guard;
    const float pwm_low_guard = (float)ctrl->cfg.pwm_min + guard;

    float ev = ctrl->v_ref - v_meas;
    float projected_pwm = (float)ctrl->cfg.pwm_neutral + gv->kp * ev + ctrl->integ_v;
    bool push_high = (projected_pwm >= pwm_high_guard) && (ev > 0.0f);
    bool push_low  = (projected_pwm <= pwm_low_guard) && (ev < 0.0f);
    if(!(push_high || push_low)){
        ctrl->integ_v += gv->ki * ev;
    }
    ctrl->integ_v = clampf(ctrl->integ_v, -0.5f*pwm_span, 0.5f*pwm_span);

    float pwm_f = (float)ctrl->cfg.pwm_neutral + gv->kp * ev + ctrl->integ_v; // ignoring kd
    if(pwm_f < (float)ctrl->cfg.pwm_min) pwm_f = (float)ctrl->cfg.pwm_min;
    if(pwm_f > (float)ctrl->cfg.pwm_max) pwm_f = (float)ctrl->cfg.pwm_max;

    // PWM斜率限制
    int16_t prev_pwm = ctrl->pwm_cmd;
    int16_t target_pwm = (int16_t)(pwm_f + 0.5f);

    int16_t delta = target_pwm - prev_pwm;
    if(delta > ctrl->cfg.pwm_slew_per_tick) delta = ctrl->cfg.pwm_slew_per_tick;
    else if(delta < -ctrl->cfg.pwm_slew_per_tick) delta = -ctrl->cfg.pwm_slew_per_tick;
    int16_t candidate_pwm = prev_pwm + delta;
    if(candidate_pwm < ctrl->cfg.pwm_min) candidate_pwm = ctrl->cfg.pwm_min;
    if(candidate_pwm > ctrl->cfg.pwm_max) candidate_pwm = ctrl->cfg.pwm_max;

    // 方向切换保护
    int16_t neutral = ctrl->cfg.pwm_neutral;
    int16_t deviation = candidate_pwm - neutral;
    int8_t desired_dir = 0;
    if(deviation > 0) desired_dir = 1;
    else if(deviation < 0) desired_dir = -1;

    int16_t abs_dev = (deviation >= 0) ? deviation : (int16_t)(-deviation);
    if(desired_dir != 0 && desired_dir != ctrl->last_dir){
        if(abs_dev < ctrl->cfg.dir_thresh_pwm){
            candidate_pwm = neutral;
            ctrl->integ_v *= 0.98f;//此处可修改积分衰减速度，如果还是有windup，可以加快衰减
        }else{
            ctrl->last_dir = desired_dir;
        }
    }else if(desired_dir == 0){
        ctrl->last_dir = 0;
    }

    ctrl->pwm_cmd = candidate_pwm;
}

// 电机PWM获取API
int16_t DepthCtrl_GetPwm(const depth_ctrl_t *ctrl){
    return ctrl? ctrl->pwm_cmd:1500; 
}

// 内环目标速度获取API
float DepthCtrl_GetVref(const depth_ctrl_t *ctrl){
    return ctrl? ctrl->v_ref:0.0f; 
}

void DepthCtrl_ResetIntegrators(depth_ctrl_t *ctrl){
    if(!ctrl) return;
    ctrl->integ_z = 0.0f;
    ctrl->integ_v = 0.0f;
    ctrl->last_dir = 0;
}
