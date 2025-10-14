#include "tasks_runtime.h"
#include "FreeRTOS.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "main.h"

/* TIM5句柄（由CubeMX生成的tim.c中定义） */
extern TIM_HandleTypeDef htim5;

/* 32位运行时计数器：高16位由软件维护，低16位由TIM5硬件计数 */
volatile uint32_t ulHighFrequencyTimerTicks = 0;

/**
 * @brief 配置TIM5用于运行时统计
 * 
 * 配置说明：
 * - TIM5是16位定时器 @ 1MHz (PSC=71)
 * - 每65.536ms溢出一次，触发中断
 * - 中断中递增高16位计数器
 * - 32位范围：4294967296 μs ≈ 71.6分钟
 * 
 * @note 此函数由FreeRTOS在vTaskStartScheduler()中自动调用
 */
void ConfigTimerForRunTimeStats(void)
{
    /* TIM5已由CubeMX配置，只需启动定时器和中断 */
    ulHighFrequencyTimerTicks = 0;
    
    /* 启动TIM5并使能更新中断 */
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief 获取32位运行时计数器值
 * 
 * @return uint32_t 当前计数值（单位：μs）
 * 
 * @note 高16位来自软件计数器，低16位来自TIM5->CNT
 * @note 此函数由FreeRTOS内核频繁调用，必须快速执行
 */
uint32_t GetRunTimeCounterValue(void)
{
    uint32_t ulTotalCount;
    uint16_t usTIMCount;
    
    /* 读取当前TIM5计数值 */
    usTIMCount = (uint16_t)TIM5->CNT;
    
    /* 组合高16位和低16位 */
    ulTotalCount = (ulHighFrequencyTimerTicks << 16) | usTIMCount;
    
    return ulTotalCount;
}

