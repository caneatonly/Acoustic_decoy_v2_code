#include "telemetry.h"
#include "console.h"
#include "time_utils.h"
#include <stdio.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/*
实现简单遥测系统，用于收集和打印关键状态变量。
使用互斥量保护共享数据，确保线程安全。
*/
static struct { 
    float z;  //滤波后深度
    float v;  //速度
    int mission_state; // 任务状态，参考 mission_manager.h 中的 mission_state_t
    int balloon_state; // 气球状态
    float v_ref; // 参考速度
    int16_t pwm; // PWM值
    float p_bag; // 气囊压力
    float p_water; // 水压
    float dPdt; // 压力变化率
    float duty; // 占空比
    float z_target; // 目标深度
} g_tlm;

// 遥测数据互斥量（定义在这里，供外部使用）
SemaphoreHandle_t g_telemetryMutex = NULL;

void Telemetry_Init(void)
{
    if (g_telemetryMutex == NULL)
    {
        g_telemetryMutex = xSemaphoreCreateMutex();
    }
}

void Telemetry_SetDepth(float z, float v, float z_target)
{
    if (xSemaphoreTake(g_telemetryMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
        g_tlm.z = z;
        g_tlm.v = v;
        g_tlm.z_target = z_target;
        xSemaphoreGive(g_telemetryMutex);
    }
}

void Telemetry_SetMissionState(int s)
{
    if (xSemaphoreTake(g_telemetryMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
        g_tlm.mission_state = s;
        xSemaphoreGive(g_telemetryMutex);
    }
}

void Telemetry_SetBalloon(int b)
{
    if (xSemaphoreTake(g_telemetryMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
        g_tlm.balloon_state = b;
        xSemaphoreGive(g_telemetryMutex);
    }
}

void Telemetry_SetControl(float v_ref, int16_t pwm)
{
    if (xSemaphoreTake(g_telemetryMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
        g_tlm.v_ref = v_ref;
        g_tlm.pwm = pwm;
        xSemaphoreGive(g_telemetryMutex);
    }
}

void Telemetry_SetPressures(float p_bag, float p_water, float dPdt, float duty)
{
    if (xSemaphoreTake(g_telemetryMutex, pdMS_TO_TICKS(5)) == pdPASS)
    {
        g_tlm.p_bag = p_bag;
        g_tlm.p_water = p_water;
        g_tlm.dPdt = dPdt;
        g_tlm.duty = duty;
        xSemaphoreGive(g_telemetryMutex);
    }
}

void Telemetry_Publish(TickType_t now_tick)
{
    // 创建本地副本，减少互斥量持有时间
    float z, v, z_target, v_ref, p_bag, p_water, dPdt, duty;
    int mission_state, balloon_state;
    int16_t pwm;
    
    if (xSemaphoreTake(g_telemetryMutex, pdMS_TO_TICKS(10)) == pdPASS)
    {
        z = g_tlm.z;
        v = g_tlm.v;
        z_target = g_tlm.z_target;
        v_ref = g_tlm.v_ref;
        mission_state = g_tlm.mission_state;
        balloon_state = g_tlm.balloon_state;
        pwm = g_tlm.pwm;
        p_bag = g_tlm.p_bag;
        p_water = g_tlm.p_water;
        dPdt = g_tlm.dPdt;
        duty = g_tlm.duty;
        xSemaphoreGive(g_telemetryMutex);
    }
    else
    {
        // 获取互斥量失败，跳过本次发布
        return;
    }
    
    // 使用线程安全的 console_printf 发布遥测数据
    const uint32_t now_ms = TimeUtils_TicksToMs(now_tick);
    console_printf("Time: %lu \r\nTLM:\r\n z=%.2f,z_target=%.2f,v=%.3f,v_ref = %.3f\r\n"
                   "mission_state = %d balloon_state = %d \r\n"
                   "pwm = %d,p_bag = %.2f,p_water = %.2f,dPdt = %.2f,duty = %.2f\r\n",
                   (unsigned long)now_ms, z, z_target, v, v_ref, 
                   mission_state, balloon_state, 
                   pwm, p_bag, p_water, dPdt, duty);
}
