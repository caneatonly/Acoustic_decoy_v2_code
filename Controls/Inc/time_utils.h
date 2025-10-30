#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取当前 FreeRTOS 节拍计数（Tick count）。
 */
static inline TickType_t TimeUtils_NowTicks(void)
{
    return xTaskGetTickCount();
}

/**
 * @brief 将节拍数转换为毫秒（毫秒时间戳）。
 */
static inline uint32_t TimeUtils_TicksToMs(TickType_t ticks)
{
    return (uint32_t)(ticks * portTICK_PERIOD_MS);
}

/**
 * @brief 将节拍差转换为秒（浮点）。
 */
static inline float TimeUtils_TicksToSeconds(TickType_t ticks)
{
    return ((float)ticks) / (float)configTICK_RATE_HZ;
}

/**
 * @brief 将毫秒转换为节拍数。
 */
static inline TickType_t TimeUtils_MsToTicks(uint32_t milliseconds)
{
    return pdMS_TO_TICKS(milliseconds);
}

#ifdef __cplusplus
}
#endif

#endif /* TIME_UTILS_H */
