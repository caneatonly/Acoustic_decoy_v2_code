// 外设层统一的基础IO模块：包含传感器读数与执行器原语（IMU/MS5837/气囊压力/电磁阀/整流罩/电机/电源/LED）

#ifndef __BSP_IO_H
#define __BSP_IO_H

#include "main.h"
#include "im948_CMD.h"
#include "FreeRTOS.h"
#include <stdint.h>
#include <stdbool.h>
#include "MS5837_lib.h"

// MS5837传感器数据结构
typedef struct {
    float temperature;       // 温度值 (°C)
    float depth;            // 深度值 (m)
    float pressure_water;   // 水下压力 (kPa,绝对压力)
    TickType_t timestamp;   // 数据时间戳 (FreeRTOS tick)
    bool data_valid;        // 数据有效性标志
} MS5837_Data_t;

// IMU传感器数据结构
typedef struct {
    float angleX;           // X轴角度 (度)
    float angleY;           // Y轴角度 (度)
    float angleZ;           // Z轴角度 (度)
    float accelX;           // X轴加速度 (m/s^2)
    float accelY;           // Y轴加速度 (m/s^2)
    float accelZ;           // Z轴加速度 (m/s^2)
    TickType_t timestamp;   // 数据时间戳 (FreeRTOS tick)
    bool data_valid;        // 数据有效性标志
} IMU_Data_t;

// 全局实例由外设层维护，不对上层暴露可写符号（使用只读 getter）

// 初始化
void SensorSystem_Init(void);
void motorInit(void);
void imuInit(void);

// IMU相关
void IMU_UpdateAngle(float angleX, float angleY, float angleZ);
void IMU_UpdateAccel(float accelX, float accelY, float accelZ);
 
// 12v外设总电源控制
void power_on(void);
void power_off(void);

// 状态灯
void LEDstatus_on(void);
void LEDstatus_off(void);

// 整流罩
void fairing_release(void);
void fairing_retract(void);

// 电磁阀
void valve_open(void);
void valve_close(void);

// 电机
void motor_test(void);
void SetMotorSpeed(uint16_t speed);

// 数据访问接口
const MS5837_Data_t* MS5837_GetData(void);
const IMU_Data_t* IMU_GetData(void);

#endif /* __BSP_IO_H */
