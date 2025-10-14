/* Console output utilities with FreeRTOS mutex protection */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdarg.h>
#include <stddef.h>



#include "FreeRTOS.h"
#include "semphr.h"

/* Global mutex handle used to protect console/UART prints */
extern SemaphoreHandle_t xConsoleMutex;

/* Initialize console mutex; safe to call once before scheduler starts. */
void console_init(void);

/* Optional convenience printf that holds the console mutex while printing. */
int console_printf(const char *fmt, ...);


#endif /* CONSOLE_H */
