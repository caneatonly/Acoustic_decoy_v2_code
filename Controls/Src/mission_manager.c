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
// 在 DEPTH_HOLD 阶段用于统计“持续处于目标附近带内”的开始时间戳（0 表示当前不在带内计时）
static uint32_t g_hold_inband_start_ms = 0;

void Mission_Init(float target_depth_m){
    g_status.state = MISSION_INIT;
    g_status.prev_state = MISSION_INIT;
    g_status.target_depth_m = target_depth_m; 
    g_status.state_enter_ms = 0; 
    g_status.started = true; 
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
        case MISSION_DEPTH_HOLD: {
            // 当“持续在目标深度附近”达到阈值后，才进入驻留阶段
            float dz = depth_m - g_status.target_depth_m;

            // 1) 超出退出阈值：立即返回 APPROACH，并重置计时
            if (fabsf(dz) > CTRL_PREP_BAND_EXIT_M) {
                g_hold_inband_start_ms = 0; // 离开带内，清零
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_APPROACH;
                g_status.state_enter_ms = now_ms;
                break;
            }

            // 2) 在带内（使用进入阈值判定）时开始或累计计时；离开带内则清零计时
            if (fabsf(dz) <= CTRL_PREP_BAND_ENTER_M) {
                if (g_hold_inband_start_ms == 0) {
                    g_hold_inband_start_ms = now_ms; // 刚进入带内，开始计时
                }
            } else {
                // 介于 ENTER 与 EXIT 之间，视为未满足“在目标附近”，清零连续计时
                g_hold_inband_start_ms = 0;
            }

            // 3) 若连续在带内的时间达到阈值，则进入 DWELL 监测阶段
            if (g_hold_inband_start_ms != 0u &&
                (uint32_t)(now_ms - g_hold_inband_start_ms) >= CTRL_HOLD_DWELL_TIME_MS) {
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_DWELL_MONITOR;
                g_status.state_enter_ms = now_ms;
                g_hold_inband_start_ms = 0; // 进入下阶段，复位该计时
            }
            break; }
        case MISSION_DWELL_MONITOR: {
            // 在驻留阶段维持深度；到时间或收到外部命令则进入回收
            // 驻留阶段深度监视，根据误差大小跳转，保障驻留期间位置稳定
            float dz = depth_m - g_status.target_depth_m;
            if (fabsf(dz) > CTRL_PREP_BAND_EXIT_M) {
                // 误差过大（超出驻留带退出阈值），重新进入APPROACH以快速拉回
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_APPROACH;
                g_status.state_enter_ms = now_ms;
                break;
            } else if (fabsf(dz) > CTRL_PREP_BAND_ENTER_M) {
                // 误差介于进入与退出阈值之间，进入HOLD以小修正
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_DEPTH_HOLD;
                g_status.state_enter_ms = now_ms;
                break;
            }
            if ((uint32_t)(now_ms - g_status.state_enter_ms) >= CTRL_RECOVERY_DELAY_MS) {
                g_status.prev_state = g_status.state;
                g_status.state = MISSION_RECOVERY_ASCEND;
                g_status.state_enter_ms = now_ms;
            }
            break; }
        case MISSION_RECOVERY_ASCEND: {
            // 表面/浅水检测：到达浅水阈值 -> 自动停机
            // 加入最小驻留时间，确保进入RECOVERY后的首次执行能完成：valve禁用/power_on/积分清零等
            if ((uint32_t)(now_ms - g_status.state_enter_ms) >= 200u) {
                if (depth_m <= CTRL_SURFACE_DEPTH_TH_M){
                    // 安全停机：中立PWM、关闭阀门与12V电源、LED心跳
                    Actuators_SetMotorPwm(1500);
                    Actuators_ValveClose();
                    Actuators_12V_PowerOff();
                    while (1) {
                        printf("Recovery complete: Surface reached, shutting down.\r\n");
                        Actuators_LedToggle();
                        HAL_Delay(1000);
                    }
                }
            }
            break; }
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
mission_status_t* Mission_GetStatus(void){
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
