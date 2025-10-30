#include "depth_estimator.h"
#include "time_utils.h"
#include <math.h>
#include <stdbool.h>

// 初始化深度估计器 设置alpha参数
void DepthEst_Init(depth_estimator_t *est, float alpha,float beta){
    if(!est) return;
    est->z_raw=0.0f; 
    est->z_filt=0.0f; 
    est->v_est=0.0f; 
    est->alpha=alpha; 
    est->beta=beta;
    est->last_update_tick = 0;
    est->valid=false; 
    est->z_prev=0.0f;
    est->vel_initialized=false;
}

void DepthEst_Update(depth_estimator_t *est, float z_meas, TickType_t now_tick){
    if(!est) return;
    if(est->last_update_tick == 0){
        est->z_raw = est->z_filt = z_meas;
        est->v_est=0.0f; 
        est->last_update_tick = now_tick;
        est->valid=true; 
        est->z_prev=z_meas; 
        est->vel_initialized=false;
        return;
    }
    TickType_t dt_ticks = now_tick - est->last_update_tick;
    if (dt_ticks == 0u) {
        return;
    }
    float dt = TimeUtils_TicksToSeconds(dt_ticks);
    est->z_raw = z_meas;
    est->z_filt += est->alpha * (z_meas - est->z_filt);
    float v_raw = (est->z_filt - est->z_prev)/dt;
    if(!est->vel_initialized){
        est->v_est = v_raw;
        est->vel_initialized = true;
    }else{
        est->v_est += est->beta * (v_raw - est->v_est);
    }
    est->z_prev = est->z_filt;
    est->last_update_tick = now_tick;
    est->valid=true;
}

// 滤波深度获取API
float DepthEst_GetDepth(const depth_estimator_t *est){
    if(est!=NULL && est->valid){
        return est->z_filt;
    }
    return 0.0f;
}

// 速度获取API
float DepthEst_GetVelocity(const depth_estimator_t *est){
    if(est!=NULL && est->valid){
        return est->v_est;
    }
    return 0.0f;
}

// 有效性检查API
bool DepthEst_IsValid(const depth_estimator_t *est){
    return est? est->valid:false;
}
