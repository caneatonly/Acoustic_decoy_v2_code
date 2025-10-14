#include "console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "projdefs.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Define the mutex handle */
SemaphoreHandle_t xConsoleMutex = NULL;

void console_init(void)
{
    if (xConsoleMutex == NULL) {
        xConsoleMutex = xSemaphoreCreateMutex();
    }
}

int console_printf(const char *fmt, ...) {
  int written = 0;
  va_list args;
  va_start(args, fmt);
  
  // If scheduler not started or mutex not created, print directly
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED || xConsoleMutex == NULL) {
    written = vprintf(fmt, args);
    va_end(args);
    return written;
  }

  // Scheduler running and mutex available: use mutex protection
  if (xSemaphoreTake(xConsoleMutex, portMAX_DELAY) == pdTRUE) {
    written = vprintf(fmt, args);
    xSemaphoreGive(xConsoleMutex);
  }
  
  va_end(args);
  return written;
}
