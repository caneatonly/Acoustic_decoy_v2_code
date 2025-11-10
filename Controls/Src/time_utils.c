#include "time_utils.h"
#include "stm32f1xx_hal.h"

static volatile BaseType_t s_scheduler_started = pdFALSE;

static TickType_t prvTicksFromHal(void)
{
    return (TickType_t)pdMS_TO_TICKS(HAL_GetTick());
}

bool TimeUtils_IsSchedulerStarted(void)
{
    return (s_scheduler_started == pdTRUE);
}

TickType_t TimeUtils_NowTicks(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
    BaseType_t state = xTaskGetSchedulerState();
    if (state == taskSCHEDULER_NOT_STARTED)
    {
        return prvTicksFromHal();
    }
#endif
    s_scheduler_started = pdTRUE;
    return xTaskGetTickCount();
}

TickType_t TimeUtils_NowTicksFromISR(void)
{
    if (s_scheduler_started == pdFALSE)
    {
        return prvTicksFromHal();
    }
    return xTaskGetTickCountFromISR();
}
