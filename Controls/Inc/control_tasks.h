#pragma once
#include "main.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <stdbool.h>


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
extern SemaphoreHandle_t g_balloonStartSem;

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

typedef enum {
	PID_LOOP_DEPTH = 0,
	PID_LOOP_VELOCITY
} pid_loop_t;

typedef enum {
	PID_MODE_APPROACH = 0,
	PID_MODE_HOLD
} pid_mode_t;

typedef enum {
	PID_TERM_KP = 0,
	PID_TERM_KI,
	PID_TERM_KD
} pid_term_t;

bool ControlTasks_IsPidTuningMode(void);
void ControlTasks_SetPidTuningMode(bool enable);
bool ControlTasks_IsOuterLoopEnabled(void);
BaseType_t ControlTasks_SetOuterLoopEnabled(bool enable, float manual_vref);
BaseType_t ControlTasks_SetManualVref(float manual_vref);
float ControlTasks_GetManualVref(void);
BaseType_t ControlTasks_UpdatePidGain(pid_loop_t loop, pid_mode_t mode, pid_term_t term, float value);
void ControlTasks_PrintPidStatus(void);
