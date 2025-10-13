#include "bsp_io.h"
#include "bsp_usart.h"
#include "im948_CMD.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "control_tasks.h"

// 全局数据实例
MS5837_Data_t g_ms5837_data = {0};
IMU_Data_t g_imu_data = {0};

// Motor control kept internal to this module
typedef struct {
    uint16_t current_pwm;   // 当前PWM占空比或频率值
    uint16_t target_pwm;    // 目标PWM值
    bool motor_enabled;     // 电机启用状态
    uint32_t last_update;   // 上次更新的时间戳
} Motor_Control_t;
static Motor_Control_t g_motor_control = {0};

// 初始化函数
void SensorSystem_Init(void)
{
    // 初始化MS5837数据
    g_ms5837_data.temperature = 0.0f;
    g_ms5837_data.depth = 0.0f;
    g_ms5837_data.timestamp = 0;
    g_ms5837_data.data_valid = false;
    g_ms5837_data.pressure_water = 0.0f;
    
    // 初始化IMU数据
    g_imu_data.angleX = 0.0f;
    g_imu_data.angleY = 0.0f;
    g_imu_data.angleZ = 0.0f;
    g_imu_data.accelX = 0.0f;
    g_imu_data.accelY = 0.0f;
    g_imu_data.accelZ = 0.0f;
    g_imu_data.timestamp = 0;
    g_imu_data.data_valid = false;
    
    // 初始化电机控制
    g_motor_control.current_pwm = 1500;  // 中性值
    g_motor_control.target_pwm = 1500;
    g_motor_control.motor_enabled = false;
    g_motor_control.last_update = HAL_GetTick();
}

// 数据访问函数
const MS5837_Data_t* MS5837_GetData(void)
{
    return &g_ms5837_data;
}

const IMU_Data_t* IMU_GetData(void)
{
    return &g_imu_data;
}

// 初始化电调
void motorInit(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
  HAL_Delay(500);
}

// 电机测试函数
void motor_test(void)
{
    // 启动电机
    g_motor_control.motor_enabled = true;
    g_motor_control.target_pwm = 2000;  // 设置目标PWM值为2000
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_motor_control.target_pwm);
    
    // 延时1秒以观察电机状态
    HAL_Delay(1000);
    
    // 停止电机
    g_motor_control.motor_enabled = false;
    g_motor_control.target_pwm = 1500;  // 设置目标PWM值为1500（中性位置）
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_motor_control.target_pwm);
}

//电机转速设置函数
void SetMotorSpeed(uint16_t speed)
{
    if (speed < 1000) speed = 1000;  // 限制最小值
    if (speed > 2000) speed = 2000;  // 限制最大值
    if(speed != 1500){
        g_motor_control.motor_enabled = true; // 启用电机
    } else {
        g_motor_control.motor_enabled = false; // 停止电机
    }
    g_motor_control.target_pwm = speed;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_motor_control.target_pwm);
}


// 初始化IMU
void imuInit(void)
{
    Cmd_03();// 1 唤醒IMU
    Cmd_12(5, 255, 0,  0, 3, 2, 2, 4, 9, 0xFFF);// 2 设置设备参数(内容1)
    Cmd_19();// 开启数据主动上报
}

  // IMU角度数据更新函数
void IMU_UpdateAngle(float angleX, float angleY, float angleZ)
{
    g_imu_data.angleX = angleX;
    g_imu_data.angleY = angleY;
    g_imu_data.angleZ = angleZ;
    g_imu_data.timestamp = HAL_GetTick();
    g_imu_data.data_valid = true;
}

// IMU加速度数据更新函数
void IMU_UpdateAccel(float accelX, float accelY, float accelZ)
{
    g_imu_data.accelX = accelX;
    g_imu_data.accelY = accelY;
    g_imu_data.accelZ = accelZ;
    g_imu_data.timestamp = HAL_GetTick();
    g_imu_data.data_valid = true;
}

void LEDstatus_on(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // PA4 LED ON
}
void LEDstatus_off(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // PA4 LED OFF
}

// 电源控制函数
void power_on(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);
}
void power_off(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
}

// 整流罩控制函数
void fairing_release(void){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
    HAL_Delay(100); // 等待100ms以确保释放完成
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
}

void fairing_retract(void){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
}

// 电磁阀控制函数
void valve_open(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
}
void valve_close(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
}
