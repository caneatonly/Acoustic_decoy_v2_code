#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H
#include <stdint.h>
#include <stdbool.h>
#include "balloon_state.h"

typedef enum {
    MISSION_INIT = 0,
    MISSION_WATER_DETECT,        //检测入水
    MISSION_APPROACH,            //进入深度预备区： 允许一定速度，进入预备深度带
    MISSION_PREP_HOLD,           //进入准备保持阶段：目标速度0，执行整流罩脱落、气囊充气
    MISSION_INFLATE_VERIFY,      //膨胀验证：确定充气完毕
    MISSION_DEPTH_HOLD,          //深度保持阶段
    MISSION_DWELL_MONITOR,       //停留监测阶段
    MISSION_RECOVERY_ASCEND,     //恢复上升阶段
    MISSION_FAILSAFE             //紧急状态
} mission_state_t;

typedef struct {
    mission_state_t state;
    mission_state_t prev_state;
    uint32_t state_enter_ms;
    float target_depth_m;
    bool started;
} mission_status_t;

void Mission_Init(float target_depth_m);
void Mission_Update(uint32_t now_ms, float depth_m, float depth_vel_mps, balloon_state_t balloon_state);
void Mission_RequestRecovery(void);
void Mission_AbortFailsafe(const char *reason);
const mission_status_t* Mission_GetStatus(void);
// Helper to move to depth hold
void Mission_GotoDepthHold(uint32_t now_ms);

// State transition helpers
// Returns true if mission state changed since last acknowledge
bool Mission_HasStateChanged(void);
// Acknowledge the state change so future calls return false until next change
void Mission_AckStateChange(void);


#endif // MISSION_MANAGER_H
