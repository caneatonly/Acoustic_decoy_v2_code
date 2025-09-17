#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/*
总配置头文件
Acoustic Decoy_v2 2025.9.17
Author: Zyshine3
Email: zyshine3@sjtu.edu.cn
*/

// Depth bands / thresholds
#define CTRL_PREP_DEPTH_TARGET_M          (5.0f)     // example placeholder
#define CTRL_PREP_BAND_IN_M               (0.30f)
#define CTRL_PREP_BAND_OUT_M              (0.40f)
#define CTRL_V_NEAR_ZERO_MPS              (0.02f)
#define CTRL_V_NEAR_ZERO_HOLD_MS          (1500u)

// Balloon stability
#define CTRL_BALLOON_DP_MARGIN_KPA        (1.0f)
#define CTRL_BALLOON_STABLE_DP_ERR_KPA    (0.3f)
#define CTRL_BALLOON_STABLE_DPDt_KPA_S    (0.5f)
#define CTRL_BALLOON_STABLE_WINDOWS       (3u)

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
#define CTRL_V_REF_MAX_APP    (0.25f)
#define CTRL_V_REF_MAX_HOLD   (0.05f)
#define CTRL_V_REF_SLEW       (0.01f)   // m/s per control step (placeholder)
#define CTRL_PWM_NEUTRAL      (1500)
#define CTRL_PWM_MIN          (1200)
#define CTRL_PWM_MAX          (1800)
#define CTRL_PWM_SLEW_PER_TICK (15)     // per 10ms tick approx

// 深度估计器 EMA 参数
#define ESTIMATOR_EMA_ALPHA_Z   (0.2f)

// Safety
#define SAFETY_DEPTH_TIMEOUT_MS   (1500u)
#define SAFETY_PRESS_TIMEOUT_MS   (1500u)

//Status Publish intervals
#define CTRL_STATUS_PUBLISH_MS    (1000u)

// 任务相关参数
#define WATER_DETECT_DEPTH_THRESHOLD_M   (0.15f)   // 超过此深度认为已入水
#define WATER_DETECT_TIMEOUT_MS          (10000u)  // 10s 超时

#endif
