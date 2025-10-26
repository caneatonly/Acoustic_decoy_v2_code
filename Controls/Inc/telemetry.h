#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

// 遥测数据互斥量（extern 声明）
extern SemaphoreHandle_t g_telemetryMutex;

void Telemetry_Init(void);
void Telemetry_Publish(uint32_t now_ms);

// 快照数据结构体数据更新API 
void Telemetry_SetDepth(float z, float v, float z_target);
void Telemetry_SetMissionState(int state);
void Telemetry_SetBalloon(int bstate);
void Telemetry_SetControl(float v_ref, int16_t pwm);
void Telemetry_SetPressures(float p_bag, float p_water, float dPdt, float duty);


#endif
