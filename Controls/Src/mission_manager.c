#include "mission_manager.h"
#include "actuators.h"
#include "control_config.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "balloon_state.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>


static mission_status_t g_status = {0};

void Mission_Init(float target_depth_m){
    g_status.state = MISSION_INIT;
    g_status.prev_state = MISSION_INIT;
    g_status.target_depth_m = target_depth_m; 
    g_status.state_enter_ms = 0; 
    g_status.started = true; // placeholder
}

// 任务模式切换逻辑
void Mission_Update(uint32_t now_ms, float depth_m, float vel_mps, balloon_state_t balloon_state){
   
    // 任务未启动则不进行状态更新
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
        case MISSION_APPROACH: {
            // 预备带进入判据：|z - z_target| <= CTRL_PREP_BAND_ENTER_M -> 进入 PREP_HOLD
            float dz = depth_m - g_status.target_depth_m;
            if (fabsf(dz) <= CTRL_PREP_BAND_ENTER_M){
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_PREP_HOLD;
                g_status.state_enter_ms = now_ms;
            }
            break; }
        case MISSION_PREP_HOLD: {
            // 预备带退出判据：|z - z_target| > CTRL_PREP_BAND_EXIT_M -> 回到 APPROACH 重新逼近
            float dz = depth_m - g_status.target_depth_m;
            if (fabsf(dz) > CTRL_PREP_BAND_EXIT_M){
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_APPROACH;
                g_status.state_enter_ms = now_ms;
                break;
            }
            // 气囊完全稳定后 -> 进入保深
            if (balloon_state == BALLOON_STABLE && fabsf(vel_mps) < CTRL_V_NEAR_ZERO_MPS) {
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_DEPTH_HOLD;
                g_status.state_enter_ms = now_ms;
            }
            break; }
        case MISSION_INFLATE_VERIFY:
        case MISSION_DEPTH_HOLD:
        case MISSION_DWELL_MONITOR:
        case MISSION_RECOVERY_ASCEND:
        case MISSION_FAILSAFE:{
            while (1) {
                printf("Mission_Stopped: Water Detect failed ,Entering Failsafe\r\n");
                Actuators_LedToggle();
                Actuators_SetMotorPwm(1500); // 停止推进器
                HAL_Delay(1000);
            }
        }
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

bool Mission_HasStateChanged(void){
    return g_status.prev_state != g_status.state;
}

void Mission_AckStateChange(void){
    // After the control loop handles one-shot actions for a new state,
    // align prev_state to state so HasStateChanged() returns false until next transition.
    g_status.prev_state = g_status.state;
}
