#pragma once
#include "main.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>


#define IMU_TX_MAX_FRAME_LEN    128U //

typedef struct
{
	uint16_t length;
	uint8_t  payload[IMU_TX_MAX_FRAME_LEN];
} ImuTxFrame_t;

extern QueueHandle_t g_imuRxQueue;
extern QueueHandle_t g_imuTxQueue;

void Task_Led(void *argument);
void Task_ImuProcess(void *argument);
void Task_ImuSend(void *argument);
void Task_UartDebug(void *argument);

void ControlTasks_Init(void);
