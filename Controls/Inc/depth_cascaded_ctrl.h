#ifndef DEPTH_CASCADED_CTRL_H
#define DEPTH_CASCADED_CTRL_H
#include <stdint.h>
#include <stdbool.h>


typedef enum { DEPTH_CTRL_MODE_APPROACH=0, 
    DEPTH_CTRL_MODE_HOLD
} depth_ctrl_mode_t;

typedef struct {
    float kp, ki, kd;
} pid_gains_t;

typedef struct {
    // 外环（深度）和内环（速度）两种模式（Approach 和 Hold）的增益
    pid_gains_t depth_app; pid_gains_t vel_app;
    pid_gains_t depth_hold; pid_gains_t vel_hold;
    float v_ref_max_app; //Approach 模式下的最大速度参考
    float v_ref_max_hold; //Hold 模式下的最大速度参考
    float v_ref_slew;        // m/s per update limit
    int16_t pwm_neutral;
    int16_t pwm_min;
    int16_t pwm_max;
    int16_t pwm_slew_per_tick; // per update limit
} depth_ctrl_config_t;

typedef struct {
    depth_ctrl_mode_t mode; //Approach or Hold
    float z_target; // 目标深度
    float v_ref;      // 速度参考
    float v_ref_prev; // 上一个速度参考
    float integ_z;  // 深度环积分
    float integ_v;  // 速度环积分
    float last_z;   // 上一个深度值
    float last_v_err; // 上一个速度误差
    int16_t pwm_cmd;  // PWM 命令
    uint32_t last_update_ms; // 上一次更新的时间戳
    depth_ctrl_config_t cfg;
    bool initialized;
} depth_ctrl_t; // 控制器状态结构体

void DepthCtrl_Init(depth_ctrl_t *ctrl, const depth_ctrl_config_t *cfg, float z_target_init);
void DepthCtrl_SetMode(depth_ctrl_t *ctrl, depth_ctrl_mode_t mode);
void DepthCtrl_SetTarget(depth_ctrl_t *ctrl, float z_target);
void DepthCtrl_Update(depth_ctrl_t *ctrl, float z_meas, float v_meas, uint32_t now_ms);
int16_t DepthCtrl_GetPwm(const depth_ctrl_t *ctrl);
float DepthCtrl_GetVref(const depth_ctrl_t *ctrl);


#endif
