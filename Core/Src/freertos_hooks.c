/* =================== FreeRTOS Hook 函数模板 =================== */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


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
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        /* 内存分配失败，通常是堆大小不够 */
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
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        /* 栈溢出错误处理，pcTaskName 为出错任务名 */
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
        (void)xTimerStart(xSafetyMonitorTimer, 0);
        portCONFIGURE_TIMER_FOR_RUN_TIME_STATS();
    
}
#endif

/* 6. 静态内存分配钩子（Idle/Timer 任务内存提供）
 * 触发条件：当启用静态内存分配时（configSUPPORT_STATIC_ALLOCATION=1），
 *            内核会调用以下函数以获取空闲任务和定时器任务所需的 TCB 与栈内存。
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
