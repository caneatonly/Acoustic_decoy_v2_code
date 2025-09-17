#include "mission_manager.h"
#include "control_config.h"
#include <string.h>
#include <stdbool.h>

// 入水判据参数


// APPROACH 阶段进入条件: 已入水
// 其余阶段占位后续补充

static mission_status_t g_status = {0};

void Mission_Init(float target_depth_m){
    g_status.state = MISSION_INIT;
    g_status.prev_state = MISSION_INIT;
    g_status.target_depth_m = target_depth_m; 
    g_status.state_enter_ms = 0; 
    g_status.started = true; // placeholder
}

// 任务模式切换逻辑
void Mission_Update(uint32_t now_ms, float depth_m, float depth_vel_mps){
    (void)depth_vel_mps;
    if(!g_status.started) return;

    switch(g_status.state){
        case MISSION_INIT:
            g_status.prev_state = g_status.state;
            g_status.state = MISSION_WATER_DETECT;
            g_status.state_enter_ms = now_ms;
            break;
        case MISSION_WATER_DETECT: {
            // 条件1: 达到深度阈值 -> 进入 APPROACH
            if(depth_m > WATER_DETECT_DEPTH_THRESHOLD_M){
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_APPROACH;
                g_status.state_enter_ms = now_ms;
            } else if((uint32_t)(now_ms - g_status.state_enter_ms) > WATER_DETECT_TIMEOUT_MS){
                // 超时进入 FAILSAFE
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_FAILSAFE;
                g_status.state_enter_ms = now_ms;
            }
            break; }
        case MISSION_APPROACH:
            // 后续添加：进入预备带检测，触发 PREP_HOLD 等
            break;
        case MISSION_PREP_HOLD:
        case MISSION_INFLATE_VERIFY:
        case MISSION_DEPTH_HOLD:
        case MISSION_DWELL_MONITOR:
        case MISSION_RECOVERY_ASCEND:
        case MISSION_FAILSAFE:
        default:
            break;
    }
}

// 外部请求进入恢复上升
void Mission_RequestRecovery(void){
    g_status.prev_state = g_status.state; 
    g_status.state = MISSION_RECOVERY_ASCEND; 
    g_status.state_enter_ms = 0;
}

// 紧急中止任务，进入 FAILSAFE
void Mission_AbortFailsafe(const char *reason){
    (void)reason; 
    g_status.prev_state = g_status.state; 
    g_status.state = MISSION_FAILSAFE; 
}

// 任务状态查询接口
const mission_status_t* Mission_GetStatus(void){
    return &g_status; 
}
