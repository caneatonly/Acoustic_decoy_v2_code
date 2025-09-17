#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" { 
#endif

// Called from main while loop periodically (e.g. every 10ms dispatcher)
void ControlLoop_Init(void);
void ControlLoop_RunIteration(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
