#include "control_tasks.h"

#include "bsp_usart.h"
#include "console.h"
#include "im948_CMD.h"
#include "usart.h"

#define IMU_RX_QUEUE_LENGTH     256U
#define IMU_TX_QUEUE_LENGTH     6U

QueueHandle_t g_imuRxQueue = NULL;
QueueHandle_t g_imuTxQueue = NULL;

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

  BaseType_t status;

  status = xTaskCreate(Task_Led, "led", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Task_ImuProcess, "imu_proc", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Task_ImuSend, "imu_tx", 384, NULL, tskIDLE_PRIORITY + 3, NULL);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Task_UartDebug, "uart_dbg", 384, NULL, tskIDLE_PRIORITY + 4, NULL);
  configASSERT(status == pdPASS);
}
