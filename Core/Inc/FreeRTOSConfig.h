/*
 * FreeRTOSConfig.h — FreeRTOS 内核配置（STM32F103，GCC/ARM_CM3）
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "tasks_runtime.h"


/* ========================= 硬件与时基 ========================= */
#define configCPU_CLOCK_HZ              ( ( unsigned long ) 72000000 )  /* CPU 主频（Hz）：用于 SysTick/时间基计算，应与 SystemClock_Config 一致。 */
#define configTICK_RATE_HZ              1000                            /* RTOS 节拍频率（Hz）：1kHz=1ms 分辨率，越高开销越大但精度更高。 */
#define configMINIMAL_STACK_SIZE        256                             /* 空闲任务最小栈深（word，Cortex-M 每 word = 4 字节）。 */
#define configTOTAL_HEAP_SIZE           (24 * 1024)                     /* 动态内存堆大小（字节）：供 pvPortMalloc 使用；与所选 heap_x.c 匹配。 */

/* =========================== 调度行为 =========================== */
#define configUSE_PREEMPTION            1                               /* 抢占式调度：高优先级就绪任务立即运行。 */
#define configUSE_TIME_SLICING          1                               /* 同优先级任务时间片轮转。 */
#define configMAX_PRIORITIES            5                               /* 优先级级数（0..configMAX_PRIORITIES-1）。 */
#define configIDLE_SHOULD_YIELD         1                               /* 有就绪任务时空闲任务主动让出。 */

/* ========================= 内核功能开关 ========================= */
#define configUSE_TIMERS                1                               /* 软件定时器。需要实现vApplicationGetTimerTaskMemory */
#define configUSE_MUTEXES               1                               /* 互斥量（含优先级继承）。 */
#define configUSE_RECURSIVE_MUTEXES      1                               /* 递归互斥量，允许同任务重复加锁。 */
#define configUSE_COUNTING_SEMAPHORES   1                               /* 计数信号量。二值信号量属于计数信号量特例，两者共同启用或弃用。 */
#define configUSE_TASK_NOTIFICATIONS    1                               /* 任务通知（轻量一对一）。 */

/* =========================== 内存管理 =========================== */
#define configSUPPORT_STATIC_ALLOCATION     1                           /* 允许静态创建（由用户提供内存）。 */
#define configSUPPORT_DYNAMIC_ALLOCATION    1                           /* 允许动态创建（使用 pvPortMalloc）。 */

/* ===================== 中断优先级映射（Cortex-M） ===================== */
#ifndef configPRIO_BITS
#define configPRIO_BITS        4                                       /* STM32F1 NVIC 优先级位宽（0..15，数值越小优先级越高）。 */
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15                  /* HAL/CMSIS 使用的“最低”中断优先级（数值最大）。 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5                   /* 允许调用 FromISR API 的最高中断优先级（数值小=更高）。 */

#define configKERNEL_INTERRUPT_PRIORITY      ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY      << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* ============================= API 选择 ============================ */
#define INCLUDE_vTaskDelay              1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskSuspend            1                       // 用于挂起任务
#define INCLUDE_xTaskGetIdleTaskHandle  1                       // 获取空闲任务句柄
#define INCLUDE_eTaskGetState           1                       // 获取任务状态
#define INCLUDE_xTaskGetSchedulerState  1                       // 获取调度器状态

/* ============================ 错误检测 =========================== */
#define configCHECK_FOR_STACK_OVERFLOW  2                               /* 栈溢出检测：2=方法2（更严格）。需实现 vApplicationStackOverflowHook。 */
#define configASSERT( x ) \
    if( ( x ) == 0 ) { \
        taskDISABLE_INTERRUPTS(); \
        for( ; ; ); \
    }                                                         /* 运行时断言：开发期早期暴露配置/调用错误。 */
#define configUSE_MALLOC_FAILED_HOOK      1                               /* 内存分配失败钩子 vApplicationMallocFailedHook。 */
#define configCHECK_HANDLER_INSTALLATION  0             /* 用于检查三个核心中断函数是否配置正确，1=启用检查，0=禁用检查 */



/* ========================= 软件定时器相关 ========================= */
#define configTIMER_TASK_PRIORITY        ( configMAX_PRIORITIES - 1 )    /* 定时器守护任务优先级。 */
#define configTIMER_TASK_STACK_DEPTH     ( configMINIMAL_STACK_SIZE )    /* 定时器守护任务栈深（word）。 */
#define configTIMER_QUEUE_LENGTH         10                              /* 定时器命令队列长度。指的是能缓存多少个 */
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0                             /* 守护任务启动钩子：适合在调度器启动后做一次性初始化（如启动软件定时器） */


/* ============================= IDLE与Tick相关 ============================= */
#define configUSE_IDLE_HOOK              0                               /* 空闲任务钩子 vApplicationIdleHook。 */
#define configUSE_TICK_HOOK              0                               /* 滴答钩子 vApplicationTickHook。 */
#define configUSE_TICKLESS_IDLE          0                               /* 启用无滴答模式 */
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP  2                         /*如果预计空闲时间大于等于2个tick，进入低功耗模式*/




/* ============================ Tick 宽度 ============================ */
#define configTICK_TYPE_WIDTH_IN_BITS   TICK_TYPE_WIDTH_32_BITS          /* 32位 Tick：@1kHz 溢出约49天。*
                                                                            同时影响事件组宽度，可设置（32-8）个标志位 */

/* ========================= 端口相关优化选项 ========================= */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1               /* Cortex-M 建议启用位图加速的就绪任务选择。 */
#endif





/* ===================== 调试 ===================== */
#define portREMOVE_STATIC_QUALIFIER     1                               /* 使部分内核静态全局对调试器可见（xRTOS 可读取任务/统计符号）。 */

#define configUSE_TRACE_FACILITY                1               /* 任务/队列等追踪元数据，供调试器/trace 使用。 */
#define configRECORD_STACK_HIGH_ADDRESS         1               /* 记录每任务栈高地址（pxEndOfStack），便于计算 Stack End/Used/Peak。 */
#define INCLUDE_uxTaskGetStackHighWaterMark     1               /* 提供栈高水位 API（字节/word 视端口而定）。 */
#define INCLUDE_uxTaskGetStackHighWaterMark2    1               /* 第二版本高水位 API（兼容不同栈增长方向）。 */
//运行时间相关
#define configGENERATE_RUN_TIME_STATS           1               /* 生成运行时统计信息（vTaskGetRunTimeStats()）。需要实现portCONFIGURE_TIMER_FOR_RUN_TIME_STATS 和portGET_RUN_TIME_COUNTER_VALUE */
#define configUSE_STATS_FORMATTING_FUNCTIONS    1               /* 启用 vTaskGetRunTimeStats()/vTaskList 文本格式化（调试辅助）。 */
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  ConfigTimerForRunTimeStats() // 实现定时器配置
#define portGET_RUN_TIME_COUNTER_VALUE()         GetRunTimeCounterValue()               // 实现获取运行时间计数器的值



#endif /* FREERTOS_CONFIG_H */
