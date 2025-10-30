#include "balloon_state.h"
#include "control_config.h"
#include <math.h>
#include <stdbool.h>

void Balloon_Init(balloon_status_t *b){
    if(!b) return; 
    b->state=BALLOON_INFLATING; 
    b->stable_windows=0; 
    b->last_transition_tick = 0;
}

void Balloon_Update(balloon_status_t *b, float duty, float dP_dt, TickType_t now_tick){
    if(!b) {return; }
    switch(b->state){
        case BALLOON_INFLATING:// 充气中，如果占空比为0且压力变化率足够小，进入稳定中
            if(duty==0.0f && fabsf(dP_dt) < CTRL_BALLOON_STABLE_DPDt_KPA_S){
                b->state=BALLOON_STABILIZING; 
                b->last_transition_tick = now_tick;
            }
            break;
        case BALLOON_STABILIZING:// 稳定中，如果占空比为0且压力变化率足够小，累计稳定窗口计数，达到要求后进入稳定完成
            if(duty==0.0f && fabsf(dP_dt) < CTRL_BALLOON_STABLE_DPDt_KPA_S){
                if(++b->stable_windows >= CTRL_BALLOON_STABLE_WINDOWS){ 
                    b->state=BALLOON_STABLE; 
                    b->last_transition_tick = now_tick;
                }
            } else { 
                b->stable_windows=0; 
            }
            break;
        case BALLOON_STABLE: default: break;
    }
}


