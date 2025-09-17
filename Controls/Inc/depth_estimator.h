#ifndef DEPTH_ESTIMATOR_H
#define DEPTH_ESTIMATOR_H
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    float z_raw;        // 原始深度测量值
    float z_filt;       // 滤波后深度
    float v_est;        // 估算的垂直速度
    float alpha;        // EMA 系数
    float z_prev;       // 上一个滤波后深度，用于速度估算
    uint32_t last_update_ms;
    bool valid;
} depth_estimator_t;

void DepthEst_Init(depth_estimator_t *est, float alpha);
void DepthEst_Update(depth_estimator_t *est, float z_meas, uint32_t now_ms);
float DepthEst_GetDepth(const depth_estimator_t *est);
float DepthEst_GetVelocity(const depth_estimator_t *est);
bool DepthEst_IsValid(const depth_estimator_t *est);


#endif // DEPTH_ESTIMATOR_H
