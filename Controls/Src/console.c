#include "console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "projdefs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

/* Define the mutex handle */
SemaphoreHandle_t xConsoleMutex = NULL;

void console_init(void)
{
    if (xConsoleMutex == NULL) {
        xConsoleMutex = xSemaphoreCreateMutex();
    }
}

int console_printf(const char *fmt, ...)
{
    if (fmt == NULL)
    {
        return 0;
    }

    va_list args;
    va_start(args, fmt);

    int written = 0;
    BaseType_t scheduler_state = xTaskGetSchedulerState();
    bool scheduler_running = (scheduler_state == taskSCHEDULER_RUNNING);
    bool in_isr = xPortIsInsideInterrupt();

    if (!scheduler_running || xConsoleMutex == NULL || in_isr)
    {
        /* Scheduler unavailable or we are in ISR context: fall back to direct print. */
        written = vprintf(fmt, args);
        va_end(args);
        return written;
    }

    if (xSemaphoreTake(xConsoleMutex, portMAX_DELAY) == pdTRUE)
    {
        written = vprintf(fmt, args);
        xSemaphoreGive(xConsoleMutex);
    }

    va_end(args);
    return written;
}
