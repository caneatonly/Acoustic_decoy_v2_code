#ifndef VALVE_CTRL_H
#define VALVE_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 参数结构体（用于查询当前控制参数）
typedef struct {
    float Kp;
    float Kd;
    float eps_kpa;             // 死区
    float dp_margin_kpa;       // 目标超水压裕量
    float guard_over_kpa;      // 超压硬保护
    uint32_t window_ms;        // 窗口时长（只读：固定为1000ms）
    uint32_t Tmin_on_ms;       // 最小开阀时间
    uint32_t Tmin_off_ms;      // 最小关阀时间
} ValveControlParams_t;

// 初始化控制算法，重置状态变量
void Valve_ControlAlgorithm_Init(void);

// 更新控制算法状态，计算并执行阀门控制，在main函数loop中调用
void Valve_ControlAlgorithm_Update(void);

// 启停充气控制（禁用时立即关闭阀门并取消任务）
void Valve_ControlAlgorithm_Enable(bool enable);
bool Valve_ControlAlgorithm_IsEnabled(void);

// 调参API
void Valve_ControlAlgorithm_SetGains(float Kp, float Kd);
void Valve_ControlAlgorithm_SetMargin(float dp_margin_kpa);
void Valve_ControlAlgorithm_SetWindow(uint32_t min_on_ms, uint32_t min_off_ms);
void Valve_ControlAlgorithm_SetEps(float eps_kpa);
void Valve_ControlAlgorithm_SetGuard(float over_kpa);
void Valve_ControlAlgorithm_GetParams(ValveControlParams_t* out);

// Telemetry getters
float Valve_GetDuty(void);
float Valve_GetPbag(void);
float Valve_GetPwater(void);
float Valve_GetdPdt(void);

// Non-blocking valve pulse scheduling is owned by valve control layer
void valve_open_for(uint32_t ms);
void valve_pulse_task(void);

// ============ PD 调参测试模式（Test Mode）API ============
// 启用后：
// - 允许通过手动设定目标压力进行充气控制（不再使用水压+裕量）
// - Mission 层可查询该模式以强制电机失能、阀控启用
void Valve_TestMode_Enable(bool enable);
bool Valve_TestMode_IsEnabled(void);
// 设定/查询手动目标压力（单位：kPa，指袋内绝对压力 P_bag 目标）
void Valve_TestMode_SetTargetKpa(float target_kpa);
float Valve_TestMode_GetTargetKpa(void);

#ifdef __cplusplus
}
#endif

#endif // VALVE_CTRL_H
