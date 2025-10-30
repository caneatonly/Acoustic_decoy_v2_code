#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "mission_manager.h"
#include "depth_cascaded_ctrl.h"
#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif


// 任务执行函数
void Mission_Execute(TickType_t now_tick, float depth_est, float velocity_est, depth_ctrl_t* ctrl, mission_status_t* s);

#ifdef __cplusplus
}
#endif
