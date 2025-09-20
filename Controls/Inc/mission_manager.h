#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H
#include <stdint.h>
#include <stdbool.h>
#include "balloon_state.h"
#include "depth_cascaded_ctrl.h"

typedef enum {
    MISSION_INIT = 0,
    MISSION_WATER_DETECT,        //1 检测入水
    MISSION_APPROACH,            //2 进入深度预备区： 允许一定速度，进入预备深度带
    MISSION_PREP_HOLD,           //3 进入准备保持阶段：目标速度0，执行整流罩脱落、气囊充气
    MISSION_DEPTH_HOLD,          //4 深度保持阶段
    MISSION_DWELL_MONITOR,       //5 停留监测阶段
    MISSION_RECOVERY_ASCEND,     //6 恢复上升阶段
    MISSION_FAILSAFE             //7 紧急状态
} mission_state_t;

typedef struct {
    mission_state_t state;       // 当前任务状态
    mission_state_t prev_state;  // 上一个任务状态
    uint32_t state_enter_ms;     // 进入当前状态的时间戳
    float target_depth_m;        // 目标深度
    bool started;                // 任务是否已启动
    bool valve_enable;           // 充气控制是否使能
    bool motor_active;           // 是否由深度控制器驱动电机
    depth_ctrl_mode_t ctrl_mode; // 深度控制器模式（接近/保持）
    bool force_vref;             // 是否强制速度参考
    float vref_cmd;              // 强制速度参考的数值
} mission_status_t;

void Mission_Init(float target_depth_m);
void Mission_Update(uint32_t now_ms, float depth_m, float depth_vel_mps, balloon_state_t balloon_state);
void Mission_RequestRecovery(void);
void Mission_AbortFailsafe(const char *reason);
mission_status_t* Mission_GetStatus(void);

// State transition helpers
// Returns true if mission state changed since last acknowledge
bool Mission_HasStateChanged(void);
// Acknowledge the state change so future calls return false until next change
void Mission_AckStateChange(void);


#endif // MISSION_MANAGER_H
