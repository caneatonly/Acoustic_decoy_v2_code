/* =================== FreeRTOS Hook 函数模板 =================== */
#include "FreeRTOS.h"
#include "console.h"
#include "task.h"
#include "timers.h"
#include <stdint.h>

/* 捕获最近一次断言信息，便于调试器查看 */
volatile const char * g_pcLastAssertFile = NULL;
volatile int g_iLastAssertLine = 0;
volatile const char * g_pcLastAssertExpr = NULL;
volatile uintptr_t g_xLastAssertValue = 0U;

void vAssertCalled( const char * pcFile, int line, const char * pcExpr, uintptr_t value )
{
    g_pcLastAssertFile = pcFile;
    g_iLastAssertLine = line;
    g_pcLastAssertExpr = pcExpr;
    g_xLastAssertValue = value;

    const char * pcTaskName = "<no-task>";
    if( xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED )
    {
        TaskHandle_t xTask = xTaskGetCurrentTaskHandle();
        if( xTask != NULL )
        {
            pcTaskName = pcTaskGetName( xTask );
        }
    }

    /* 尽可能输出调试信息（若控制台已初始化）。 */
    console_printf("\r\n*** FreeRTOS ASSERT ***\r\nfile: %s\r\nline: %d\r\nexpr: %s\r\nvalue: 0x%08lX\r\ntask: %s\r\n", pcFile, line, pcExpr, ( unsigned long ) value, pcTaskName );

    taskDISABLE_INTERRUPTS();

    /* 触发 BKPT 方便调试器捕获。 */
    __asm volatile ( "bkpt 0" );

    for( ;; )
    {
        __asm volatile ( "nop" );
    }
}


/* 1. 空闲任务钩子（Idle Hook）
 * 触发条件：当系统没有就绪任务时，由空闲任务调用
 * 注意：Idle Hook 必须非常短小，不能调用阻塞 API
 */
#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook(void)
{
    /* 可以在这里进入低功耗模式、省电操作等 */
}
#endif


/* 2. 堆内存不足钩子（Malloc Failed Hook）
 * 触发条件：pvPortMalloc 内存分配失败
 */
#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook(void)
{
    /* 输出调试信息（如果串口已初始化） */

    console_printf("FATAL: Heap allocation failed! Increase configTOTAL_HEAP_SIZE\r\n");

    
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        /* 内存分配失败，通常是堆大小不够 */
        /* 建议：增加 FreeRTOSConfig.h 中的 configTOTAL_HEAP_SIZE */
    }
}
#endif

/* 3. 系统滴答钩子（Tick Hook）
 * 触发条件：每次系统 Tick 中断都会执行
 * 注意：运行在中断上下文，不能调用会引起阻塞的 API
 */
#if (configUSE_TICK_HOOK == 1)
void vApplicationTickHook(void)
{
    /* 可用于周期性检查、软件定时器等 */
}
#endif

/* 4. 栈溢出钩子（Stack Overflow Hook）
 * 触发条件：任务栈检测到溢出（需 configCHECK_FOR_STACK_OVERFLOW 开启）
 * 注意：不同模式检测开销不同
 */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    ( void ) xTask;

    /* 禁用中断防止进一步损坏 */
    taskDISABLE_INTERRUPTS();
    
    /* 记录错误信息（如果有调试输出） */
    #ifdef DEBUG
    console_printf("Stack overflow in task: %s\r\n", pcTaskName);
    #endif
    
    /* 可选：点亮 LED 或设置错误标志 */
    // HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
    
    /* 可选：记录到非易失性存储器 */
    // log_error_to_flash(ERROR_STACK_OVERFLOW, pcTaskName);
    
    /* 死循环等待看门狗复位或调试 */
    for (;;)
    {
        /* 防止编译器优化掉空循环 */
        __asm volatile ("nop");
    }
}
#endif

/* 5. 守护任务钩子（Daemon Task Startup Hook）
 * 触发条件：软件定时器服务/守护任务启动时调用
 * 注意：这个 Hook 在 vTaskStartScheduler 后，timer service task 第一次运行前执行
 */
#if (configUSE_DAEMON_TASK_STARTUP_HOOK == 1)
void vApplicationDaemonTaskStartupHook(void)
{
    /* 在调度器启动后，启动软件定时器等一次性初始化 */
    /* 注意：不要在这里启动未创建的定时器！会导致断言失败 */
    // (void)xTimerStart(xSafetyMonitorTimer, 0);  // BUG: xSafetyMonitorTimer 未定义！
    portCONFIGURE_TIMER_FOR_RUN_TIME_STATS();
}
#endif

/* 6. 静态内存分配钩子（Idle/Timer 任务内存提供）
 * 触发条件：当启用静态内存分配时（configSUPPORT_STATIC_ALLOCATION=1），
 *            内核会调用以下函数以获取空闲任务和定时器任务所需的 TCB 与栈内存。
 * 即在此处需要手动提供IDLE和定时器任务的TCB和栈空间。
 */
#if (configSUPPORT_STATIC_ALLOCATION == 1)

/* 提供 Idle 任务的 TCB 和栈缓冲区 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *pulIdleTaskStackSize)
{
    /* 使用静态存储，避免放在栈上 */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = (configSTACK_DEPTH_TYPE)configMINIMAL_STACK_SIZE;
}

/* 提供 Timer Service 任务的 TCB 和栈缓冲区 */
#if (configUSE_TIMERS == 1)
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = (configSTACK_DEPTH_TYPE)configTIMER_TASK_STACK_DEPTH;
}
#endif /* configUSE_TIMERS */

#endif /* configSUPPORT_STATIC_ALLOCATION */
