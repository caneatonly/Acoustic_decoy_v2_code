#include "control_tasks.h"

#include "bsp_usart.h"
#include "console.h"
#include "im948_CMD.h"
#include "usart.h"
#include "MS5837_lib.h"
#include "i2c.h"

#define IMU_RX_QUEUE_LENGTH     256U
#define IMU_TX_QUEUE_LENGTH     6U

QueueHandle_t g_imuRxQueue = NULL;
QueueHandle_t g_imuTxQueue = NULL;
SemaphoreHandle_t g_ms5837DataMutex = NULL;

void Task_Led(void *argument)
{
  (void)argument;
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Task_ImuProcess(void *argument)
{
  (void)argument;
  uint8_t byte = 0U;

  for (;;)
  {
    if (xQueueReceive(g_imuRxQueue, &byte, portMAX_DELAY) == pdPASS)
    {
      Cmd_GetPkt(byte);

      while (xQueueReceive(g_imuRxQueue, &byte, 0) == pdPASS)
      {
        Cmd_GetPkt(byte);
      }
    }
  }
}

void Task_ImuSend(void *argument)
{
  (void)argument;
  ImuTxFrame_t frame;

  for (;;)
  {
    if (xQueueReceive(g_imuTxQueue, &frame, portMAX_DELAY) == pdPASS)
    {
      (void)HAL_UART_Transmit(&huart2, frame.payload, frame.length, HAL_MAX_DELAY);
    }
  }
}

void Task_UartDebug(void *argument)
{
  (void)argument;
  for (;;)
  {
    UART1_DataHandler();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief MS5837压力传感器数据采集任务
 * @param argument 任务参数（未使用）
 * 
 * 功能说明:
 * - 周期性调用MS5837状态机处理函数
 * - MS5837完整采样周期: ~50ms (D1转换19ms + D2转换19ms + 通信开销)
 * - 状态机内部使用vTaskDelay实现ADC转换等待
 * - 最终数据更新率: 约20Hz (每50ms一组新数据)
 * - 数据更新到全局变量 g_ms5837_data (互斥量保护)
 * 
 * 注意:
 * - 不使用 vTaskDelayUntil 是因为状态机内部已经有延时控制
 * - 使用快速轮询模式，让状态机自主控制时序
 */
void Task_MS5837Process(void *argument)
{
  (void)argument;
  
  for (;;)
  {
    // 调用MS5837状态机处理函数
    // 状态机会根据当前状态自动处理延时（WAIT状态会调用vTaskDelay）
    MS5837_Process(&hi2c1, &MS5837_info_t);
    
    // 短暂延时，避免在非WAIT状态时CPU占用过高
    // 对于需要等待的状态（WAIT_CONVERT），状态机内部会调用vTaskDelay(20ms)
    // 对于快速状态（START/READ/CALCULATE），这里的5ms延时防止任务饿死其他低优先级任务
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void ControlTasks_Init(void)
{
  // Initialize console mutex before creating tasks
  console_init();

  if (g_imuRxQueue == NULL)
  {
    g_imuRxQueue = xQueueCreate(IMU_RX_QUEUE_LENGTH, sizeof(uint8_t));
    configASSERT(g_imuRxQueue != NULL);
  }

  if (g_imuTxQueue == NULL)
  {
    g_imuTxQueue = xQueueCreate(IMU_TX_QUEUE_LENGTH, sizeof(ImuTxFrame_t));
    configASSERT(g_imuTxQueue != NULL);
  }

  // 创建MS5837数据保护互斥量
  if (g_ms5837DataMutex == NULL)
  {
    g_ms5837DataMutex = xSemaphoreCreateMutex();
    configASSERT(g_ms5837DataMutex != NULL);
  }

  BaseType_t status;

  status = xTaskCreate(Task_Led, "led", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Task_ImuProcess, "imu_proc", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Task_ImuSend, "imu_tx", 384, NULL, tskIDLE_PRIORITY + 3, NULL);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Task_UartDebug, "uart_dbg", 384, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);

  // 创建MS5837任务 - 优先级低于IMU，高于LED
  status = xTaskCreate(Task_MS5837Process, "ms5837", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
  configASSERT(status == pdPASS);
}
