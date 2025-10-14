# FreeRTOS Runtime Statistics Configuration for STM32F103RCT6

## 问题分析

STM32F103RCT6的**所有定时器都是16位**，包括TIM5。之前错误地认为TIM5是32位定时器。

## 解决方案：使用TIM5 + 中断扩展为32位

### CubeMX配置步骤

1. **打开 Acoustic_decoy_v2.ioc**

2. **Pinout & Configuration → Timers → TIM5**
   - Clock Source: `Internal Clock`
   
3. **Configuration → Parameter Settings**:
   ```
   Prescaler (PSC - 16 bits value): 71
   Counter Mode: Up
   Counter Period (AutoReload Register - 16 bits value): 65535
   Internal Clock Division (CKD): No Division
   auto-reload preload: Enable
   ```
   
4. **Configuration → NVIC Settings**:
   ```
   ✅ TIM5 global interrupt
   Preemption Priority: 7
   Sub Priority: 0
   ```
   **注意**: Priority必须 ≥ 6，因为configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5

5. **Project Manager → Advanced Settings**:
   - 确保 TIM5 使用 HAL 库

6. **点击 "GENERATE CODE"**

### 代码修改

#### 1. tasks_runtime.h
```c
#ifndef TASKS_RUNTIME_H
#define TASKS_RUNTIME_H

#include <stdint.h>

void ConfigTimerForRunTimeStats(void);
uint32_t GetRunTimeCounterValue(void);

#endif
```

#### 2. tasks_runtime.c
```c
#include "tasks_runtime.h"
#include "FreeRTOS.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "main.h"

/* TIM5句柄（由CubeMX生成的tim.c中定义） */
extern TIM_HandleTypeDef htim5;

/* 32位运行时计数器：高16位由软件维护，低16位由TIM5硬件计数 */
static volatile uint32_t ulHighFrequencyTimerTicks = 0;

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

/**
 * @brief TIM5更新中断回调函数
 * 
 * @param htim 定时器句柄
 * 
 * @note 每65.536ms调用一次（TIM5溢出）
 * @note 此函数在中断中执行，必须快速返回
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        /* TIM5溢出，递增高16位计数器 */
        ulHighFrequencyTimerTicks++;
    }
}
```

#### 3. stm32f1xx_it.c（CubeMX已生成，确认存在）
```c
/* 在 USER CODE BEGIN 1 */
extern TIM_HandleTypeDef htim5;
/* USER CODE END 1 */

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}
```

### 验证步骤

1. 编译项目
2. 烧录程序
3. 在串口输入 `tim5` 查看TIM5状态
4. 输入 `stats` 查看任务运行时统计

### 预期结果

- TIM5->CNT 应该在 0-65535 之间快速增长（每秒增加1,000,000）
- Runtime百分比总和应该接近100%
- IDLE任务应该占80-95%（系统空闲时）

### 时间分辨率

- **Tick分辨率**: 1μs
- **溢出时间**: 4294秒 ≈ 71.6分钟
- **统计精度**: 足够精确，满足FreeRTOS运行时统计需求

## 为什么不直接使用SysTick？

FreeRTOS要求运行时统计计数器的频率**至少是Tick频率的10倍**：
- FreeRTOS Tick: 1kHz (1ms)
- 要求Runtime Counter: ≥10kHz
- 我们的配置: 1MHz (远超要求，提供μs级精度)

## 替代方案（如果不想使用中断）

可以使用DWT CYCCNT寄存器（Cortex-M3内置）：

```c
// FreeRTOSConfig.h
#define configGENERATE_RUN_TIME_STATS 1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() do { \
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
    DWT->CYCCNT = 0; \
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
} while(0)
#define portGET_RUN_TIME_COUNTER_VALUE() (DWT->CYCCNT / 72)  // 72MHz → 1MHz
```

优点：无需额外定时器，无中断开销
缺点：CYCCNT是32位@72MHz，约60秒溢出，需要更频繁处理溢出
