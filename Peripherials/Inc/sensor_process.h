#ifndef __SENSOR_PROCESS_H
#define __SENSOR_PROCESS_H

#include "main.h"
#include "im948_CMD.h"
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    float temperature;       // 温度值 (°C)
    float depth;            // 深度值 (m)
    uint32_t timestamp;     // 数据时间戳 (系统tick)
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
    uint32_t timestamp;     // 数据时间戳
    bool data_valid;        // 数据有效性标志
} IMU_Data_t;

// 电机控制数据结构
typedef struct {
    uint16_t current_pwm;   // 当前PWM值
    uint16_t target_pwm;    // 目标PWM值
    bool motor_enabled;     // 电机使能状态
    uint32_t last_update;   // 最后更新时间
} Motor_Control_t;

// 全局实例声明
extern MS5837_Data_t g_ms5837_data;
extern IMU_Data_t g_imu_data;
extern Motor_Control_t g_motor_control;

// 传感器处理函数声明
void SensorSystem_Init(void);
void ProcessIMUData(void);
void motorInit(void);
void imuInit(void);
void ProcessUart3Data(uint8_t *data);
void IMU_UpdateAngle(float angleX, float angleY, float angleZ);
void IMU_UpdateAccel(float accelX, float accelY, float accelZ);

//状态灯控制函数
void LEDstatus_on(void);
void LEDstatus_off(void);

// 数据访问接口
MS5837_Data_t* MS5837_GetData(void);
IMU_Data_t* IMU_GetData(void);
Motor_Control_t* Motor_GetData(void);

#endif /* __SENSOR_PROCESS_H */
