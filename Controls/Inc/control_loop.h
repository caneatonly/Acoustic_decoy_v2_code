#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#ifdef __cplusplus
extern "C" { 
#endif

// 由main loop调用
void ControlLoop_Init(void);
void ControlLoop_RunIteration(TickType_t now_tick);

#ifdef __cplusplus
}
#endif
