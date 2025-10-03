#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/*
总配置头文件
Acoustic Decoy_v2 2025.9.17
Author: Zyshine3
Email: zyshine3@sjtu.edu.cn
*/

// Depth bands / thresholds
#define CTRL_DEPTH_TARGET_M             (5.0f)     // 目标深度
#define CTRL_PREP_BAND_ENTER_M                (0.30f)    // 进入深度预备区阈值
#define CTRL_PREP_BAND_EXIT_M                 (0.45f)    // 退出深度预备区阈值
#define CTRL_V_NEAR_ZERO_MPS            (0.02f)   // 速度接近零阈值，用于判断是否进入HOLD
 

// Balloon stability
#define CTRL_BALLOON_DP_MARGIN_KPA        (1.0f)
#define CTRL_BALLOON_STABLE_DP_ERR_KPA    (0.3f)
#define CTRL_BALLOON_STABLE_DPDt_KPA_S    (0.5f)
#define CTRL_BALLOON_STABLE_WINDOWS       (3u)
// Inflation completion criterion
#define CTRL_BALLOON_DUTY_DONE_TH         (0.05f)

// PID defaults (占位！！需要进行参数整定)
// Approach模式下 串级PID增益
#define PID_Z_KP_APP   (0.8f)
#define PID_Z_KI_APP   (0.0f)
#define PID_Z_KD_APP   (0.0f)
#define PID_V_KP_APP   (0.6f)
#define PID_V_KI_APP   (0.0f)
#define PID_V_KD_APP   (0.0f)
// Hold模式下 串级PID增益
#define PID_Z_KP_HOLD  (1.2f)
#define PID_Z_KI_HOLD  (0.05f)
#define PID_Z_KD_HOLD  (0.0f)
#define PID_V_KP_HOLD  (0.8f)
#define PID_V_KI_HOLD  (0.02f)
#define PID_V_KD_HOLD  (0.0f)

// Ramps / limits
#define CTRL_V_REF_MAX_APP    (0.25f)   // APPROACH 模式下最大参考速度
#define CTRL_V_REF_MAX_HOLD   (0.05f)   // HOLD 模式下最大参考速度
#define CTRL_V_REF_SLEW       (0.01f)   // m/s per control step (placeholder)
#define CTRL_PWM_NEUTRAL      (1500)    // 中立PWM
#define CTRL_PWM_MIN          (1000)    // 最小PWM
#define CTRL_PWM_MAX          (2000)    // 最大PWM
#define CTRL_PWM_SLEW_PER_TICK (15)     // 每次控制循环最大PWM变化量

// 深度估计器 EMA 参数
#define ESTIMATOR_EMA_ALPHA_Z   (0.2f)

// Safety
#define SAFETY_DEPTH_TIMEOUT_MS   (1500u)
#define SAFETY_PRESS_TIMEOUT_MS   (1500u)

//状态上报间隔
#define CTRL_STATUS_PUBLISH_MS    (1000u)

// 任务相关参数
#define WATER_DETECT_DEPTH_THRESHOLD_M   (0.15f)   // 超过此深度认为已入水
#define WATER_DETECT_TIMEOUT_MS          (10000u)  // 10s 超时

// 保深驻留与回收参数
// 在进入 MISSION_DEPTH_HOLD 后持续该时长（且保持在预备带内），即认为已到位，进入驻留/省电阶段
#define CTRL_HOLD_DWELL_TIME_MS          (60000u)  // 60s，可调
// 在驻留阶段停留该时长后自动进入回收上浮（如果没有外部上浮命令提前触发）
#define CTRL_RECOVERY_DELAY_MS           (300000u) // 5min，可调
// 上浮阶段的目标速度（负号代表上行，单位 m/s），由深度控制器产生推力
#define CTRL_RECOVERY_ASCEND_VREF        (-0.20f)

// 表面检测与自动停机（RECOVERY结束判据）
#define CTRL_SURFACE_DEPTH_TH_M          (0.10f)   // 小于等于该深度视为到达水面/浅水

#endif
