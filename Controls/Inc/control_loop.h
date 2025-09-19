#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" { 
#endif

// 由main loop调用
void ControlLoop_Init(void);
void ControlLoop_RunIteration(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
