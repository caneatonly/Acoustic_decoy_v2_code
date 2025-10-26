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

// IMU and sensor data mutexes/queues
extern QueueHandle_t g_imuRxQueue;
extern QueueHandle_t g_imuTxQueue;
extern SemaphoreHandle_t g_ms5837DataMutex;

// Control loop shared data mutexes
extern SemaphoreHandle_t g_depthEstMutex;
extern SemaphoreHandle_t g_depthCtrlMutex;
extern SemaphoreHandle_t g_balloonMutex;

// Task function declarations
void Task_Led(void *argument);
void Task_ImuProcess(void *argument);
void Task_ImuSend(void *argument);
void Task_UartDebug(void *argument);
void Task_MS5837Process(void *argument);
void Task_DepthEstimator(void *argument);
void Task_BalloonStateMachine(void *argument);
void Task_Telemetry(void *argument);
void Task_MissionManager(void *argument);
void Task_MissionExecutor(void *argument);

void ControlTasks_Init(void);
