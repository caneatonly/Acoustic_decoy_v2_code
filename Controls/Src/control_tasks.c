#include "control_tasks.h"



void Task_Led(void *argument) {
  (void)argument;
  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
