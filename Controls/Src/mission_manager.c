#include "mission_manager.h"
#include "actuators.h"
#include "control_config.h"
#include "console.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "balloon_state.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>



static mission_status_t g_status = {0};
static SemaphoreHandle_t g_statusMutex = NULL;
static void MissionEnsureMutex(void)
{
    if (g_statusMutex == NULL)
    {
        g_statusMutex = xSemaphoreCreateRecursiveMutex();
        configASSERT(g_statusMutex != NULL);
    }
}
// 在 DEPTH_HOLD 阶段用于统计“持续处于目标附近带内”的开始时间戳（0 表示当前不在带内计时）
static TickType_t g_hold_inband_start_tick = 0;

mission_status_t* Mission_LockStatus(TickType_t timeout_ticks)
{
    MissionEnsureMutex();
    if (xSemaphoreTakeRecursive(g_statusMutex, timeout_ticks) == pdPASS)
    {
        return &g_status;
    }
    return NULL;
}

void Mission_UnlockStatus(void)
{
    configASSERT(g_statusMutex != NULL);
    (void)xSemaphoreGiveRecursive(g_statusMutex);
}

mission_status_t* Mission_GetStatus(void)
{
    return &g_status;
}

void Mission_Init(float target_depth_m){
    MissionEnsureMutex();
    mission_status_t *status = Mission_LockStatus(portMAX_DELAY);
    configASSERT(status != NULL);
    status->state = MISSION_INIT;
    status->prev_state = MISSION_INIT;
    status->target_depth_m = target_depth_m; 
    status->state_enter_tick = 0; 
    status->started = true; 
    Mission_UnlockStatus();
}

// 任务模式切换逻辑
void Mission_Update(TickType_t now_tick, float depth_m, float vel_mps, balloon_state_t balloon_state){

    mission_status_t *status = Mission_LockStatus(portMAX_DELAY);
    configASSERT(status != NULL);
    // 任务未启动则不进行状态更新
    if(!status->started)
    {
        Mission_UnlockStatus();
        return;
    }

    switch(status->state){
        case MISSION_INIT:
            status->prev_state = status->state;
            status->state = MISSION_WATER_DETECT;
            status->state_enter_tick = now_tick;
            break;
        case MISSION_WATER_DETECT: {
            // 条件: 达到深度阈值 -> 进入 APPROACH
            if(depth_m > WATER_DETECT_DEPTH_THRESHOLD_M){
                status->prev_state = status->state;
                status->state = MISSION_APPROACH;
                status->state_enter_tick = now_tick;
            } else if((TickType_t)(now_tick - status->state_enter_tick) > pdMS_TO_TICKS(WATER_DETECT_TIMEOUT_MS)){
                // 超时进入 FAILSAFE
                status->prev_state = status->state;
                status->state = MISSION_FAILSAFE;
                status->state_enter_tick = now_tick;
            }

            break; }
        case MISSION_APPROACH: {
            // 预备带进入判据：|z - z_target| <= CTRL_PREP_BAND_ENTER_M -> 进入 PREP_HOLD
            float dz = depth_m - status->target_depth_m;
            if (fabsf(dz) <= CTRL_PREP_BAND_ENTER_M){
                status->prev_state = status->state;
                status->state = MISSION_PREP_HOLD;
                status->state_enter_tick = now_tick;
            }
            break; }
        case MISSION_PREP_HOLD: {
            // 预备带退出判据：|z - z_target| > CTRL_PREP_BAND_EXIT_M -> 回到 APPROACH 重新逼近
            float dz = depth_m - status->target_depth_m;
            if (fabsf(dz) > CTRL_PREP_BAND_EXIT_M){
                status->prev_state = status->state;
                status->state = MISSION_APPROACH;
                status->state_enter_tick = now_tick;
                break;
            }
            // 气囊完全稳定后 -> 进入保深
            if (balloon_state == BALLOON_STABLE && fabsf(vel_mps) < CTRL_V_NEAR_ZERO_MPS) {
                status->prev_state = status->state;
                status->state = MISSION_DEPTH_HOLD;
                status->state_enter_tick = now_tick;
            }
            break; }
        case MISSION_DEPTH_HOLD: {
            // 当“持续在目标深度附近”达到阈值后，才进入驻留阶段
            float dz = depth_m - status->target_depth_m;

            // 1) 超出退出阈值：立即返回 APPROACH，并重置计时
            if (fabsf(dz) > CTRL_PREP_BAND_EXIT_M) {
                g_hold_inband_start_tick = 0; // 离开带内，清零
                status->prev_state = status->state;
                status->state = MISSION_APPROACH;
                status->state_enter_tick = now_tick;
                break;
            }

            // 2) 在带内（使用进入阈值判定）时开始或累计计时；离开带内则清零计时
            if (fabsf(dz) <= CTRL_PREP_BAND_ENTER_M) {
                if (g_hold_inband_start_tick == 0) {
                    g_hold_inband_start_tick = now_tick; // 刚进入带内，开始计时
                }
            } else {
                // 介于 ENTER 与 EXIT 之间，视为未满足“在目标附近”，清零连续计时
                g_hold_inband_start_tick = 0;
            }

            // 3) 若连续在带内的时间达到阈值，则进入 DWELL 监测阶段
            if (g_hold_inband_start_tick != 0u &&
                (TickType_t)(now_tick - g_hold_inband_start_tick) >= pdMS_TO_TICKS(CTRL_HOLD_DWELL_TIME_MS)) {
                status->prev_state = status->state;
                status->state = MISSION_DWELL_MONITOR;
                status->state_enter_tick = now_tick;
                g_hold_inband_start_tick = 0; // 进入下阶段，复位该计时
            }
            break; }
        case MISSION_DWELL_MONITOR: {
            // 在驻留阶段维持深度；到时间或收到外部命令则进入回收
            // 驻留阶段深度监视，根据误差大小跳转，保障驻留期间位置稳定
            float dz = depth_m - status->target_depth_m;
            if (fabsf(dz) > CTRL_PREP_BAND_EXIT_M) {
                // 误差过大（超出驻留带退出阈值），重新进入APPROACH以快速拉回
                status->prev_state = status->state;
                status->state = MISSION_APPROACH;
                status->state_enter_tick = now_tick;
                break;
            } else if (fabsf(dz) > CTRL_PREP_BAND_ENTER_M) {
                // 误差介于进入与退出阈值之间，进入HOLD以小修正
                status->prev_state = status->state;
                status->state = MISSION_DEPTH_HOLD;
                status->state_enter_tick = now_tick;
                break;
            }
            if ((TickType_t)(now_tick - status->state_enter_tick) >= pdMS_TO_TICKS(CTRL_RECOVERY_DELAY_MS)) {
                status->prev_state = status->state;
                status->state = MISSION_RECOVERY_ASCEND;
                status->state_enter_tick = now_tick;
            }
            break; }
        case MISSION_RECOVERY_ASCEND: {
            // 表面/浅水检测：到达浅水阈值 -> 自动停机
            // 加入最小驻留时间，确保进入RECOVERY后的首次执行能完成：valve禁用/power_on/积分清零等
            if ((TickType_t)(now_tick - status->state_enter_tick) >= pdMS_TO_TICKS(200u)) {
                if (depth_m <= CTRL_SURFACE_DEPTH_TH_M){
                    status->prev_state = status->state;
                    status->state = MISSION_RECOVERY_SHUTDOWN;
                    status->state_enter_tick = now_tick;
                }
            }
            break; }
        case MISSION_RECOVERY_SHUTDOWN:
            // 停机阶段由 Mission_Execute 统一处理
            break;
        case MISSION_FAILSAFE:
            // 紧急状态的动作在 Mission_Execute 中处理
            break;
        default:
            break;
    }

    Mission_UnlockStatus();
}

// 外部请求进入恢复上升
void Mission_RequestRecovery(void){
    mission_status_t *status = Mission_LockStatus(portMAX_DELAY);
    if (status != NULL)
    {
        status->prev_state = status->state; 
        status->state = MISSION_RECOVERY_ASCEND; 
        status->state_enter_tick = xTaskGetTickCount();
        Mission_UnlockStatus();
    }
}

// 紧急中止任务，进入 FAILSAFE
void Mission_AbortFailsafe(const char *reason){
    (void)reason; 
    mission_status_t *status = Mission_LockStatus(portMAX_DELAY);
    if (status != NULL)
    {
        status->prev_state = status->state; 
        status->state = MISSION_FAILSAFE; 
        status->state_enter_tick = xTaskGetTickCount();
        Mission_UnlockStatus();
    }
}

bool Mission_HasStateChanged(void){
    bool changed = false;
    mission_status_t *status = Mission_LockStatus(0);
    if (status != NULL)
    {
        changed = (status->prev_state != status->state);
        Mission_UnlockStatus();
    }
    return changed;
}

void Mission_AckStateChange(void){
    // After the control loop handles one-shot actions for a new state,
    // align prev_state to state so HasStateChanged() returns false until next transition.
    mission_status_t *status = Mission_LockStatus(0);
    if (status != NULL)
    {
        status->prev_state = status->state;
        Mission_UnlockStatus();
    }
}

void Mission_SetTargetDepth(float depth_m) {
    mission_status_t *status = Mission_LockStatus(portMAX_DELAY);
    if (status != NULL) {
        status->target_depth_m = depth_m;
        Mission_UnlockStatus();
    }
}
