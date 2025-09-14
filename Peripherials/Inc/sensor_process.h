//该头文件包含了传感器数据处理的相关函数声明和数据结构定义
//包括 IMU，MS5837，电磁阀，整流罩释放机构

#ifndef __SENSOR_PROCESS_H
#define __SENSOR_PROCESS_H

#include "main.h"
#include "im948_CMD.h"
#include <stdint.h>
#include <stdbool.h>
#include "MS5837_lib.h"

// MS5837传感器数据结构
typedef struct {
    float temperature;       // 温度值 (°C)
    float depth;            // 深度值 (m)
    float pressure_water;   // 水下压力 (kPa,绝对压力)
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

// 传感器初始化函数
void SensorSystem_Init(void);
void motorInit(void);
void imuInit(void);
// void ProcessUart3Data(uint8_t *data);

// IMU相关函数
void ProcessIMUData(void);
void IMU_UpdateAngle(float angleX, float angleY, float angleZ);
void IMU_UpdateAccel(float accelX, float accelY, float accelZ);
 
// 12v外设总电源控制函数
void power_on(void);
void power_off(void);

// 状态灯控制函数
void LEDstatus_on(void);
void LEDstatus_off(void);

// 整流罩控制函数
void fairing_release(void);
void fairing_retract(void);

// 电磁阀控制函数
void valve_open(void);
void valve_close(void);

// 电机控制函数函数
void motor_test(void); // 电机测试函数
void SetMotorSpeed(uint16_t speed); // 电机速度设置函数/

// 数据访问接口
MS5837_Data_t* MS5837_GetData(void);
IMU_Data_t* IMU_GetData(void);
Motor_Control_t* Motor_GetData(void);

#endif /* __SENSOR_PROCESS_H */
