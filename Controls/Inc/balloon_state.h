#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#ifdef __cplusplus
extern "C" { 
#endif

//气囊状态枚举
typedef enum { BALLOON_INFLATING=0, 
    BALLOON_STABILIZING, 
    BALLOON_STABLE 
} balloon_state_t;

typedef struct {
    balloon_state_t state;
    uint8_t stable_windows;
    TickType_t last_transition_tick;
} balloon_status_t;

void Balloon_Init(balloon_status_t *b);
void Balloon_Update(balloon_status_t *b, float duty, float dP_dt, TickType_t now_tick);


#ifdef __cplusplus
}
#endif
