 #include "depth_estimator.h"
 #include <math.h>
 #include <stdbool.h>

// 初始化深度估计器 设置alpha参数
void DepthEst_Init(depth_estimator_t *est, float alpha){
    if(!est) return;
    est->z_raw=0.0f; 
    est->z_filt=0.0f; 
    est->v_est=0.0f; 
    est->alpha=alpha; 
    est->last_update_ms=0; 
    est->valid=false; 
    est->z_prev=0.0f;
}

void DepthEst_Update(depth_estimator_t *est, float z_meas, uint32_t now_ms){
    if(!est) return;
    if(est->last_update_ms == 0){
        est->z_raw = est->z_filt = z_meas;
        est->v_est=0.0f; 
        est->last_update_ms=now_ms; 
        est->valid=true; 
        est->z_prev=z_meas; 
        return;
    }
    uint32_t dt_ms = now_ms - est->last_update_ms;
    if(dt_ms==0) return;
    float dt = dt_ms*0.001f;
    est->z_raw = z_meas;
    est->z_filt += est->alpha * (z_meas - est->z_filt);
    est->v_est = (est->z_filt - est->z_prev)/dt;
    est->z_prev = est->z_filt;
    est->last_update_ms = now_ms; 
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
