#include "sensor_process.h"
#include "bsp_usart.h"
#include "im948_CMD.h"
#include "tim.h"

// 全局数据实例
MS5837_Data_t g_ms5837_data = {0};
IMU_Data_t g_imu_data = {0};
Motor_Control_t g_motor_control = {0};

// 初始化函数
void SensorSystem_Init(void)
{
    // 初始化MS5837数据
    g_ms5837_data.temperature = 0.0f;
    g_ms5837_data.depth = 0.0f;
    g_ms5837_data.timestamp = 0;
    g_ms5837_data.data_valid = false;
    
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
MS5837_Data_t* MS5837_GetData(void)
{
    return &g_ms5837_data;
}

IMU_Data_t* IMU_GetData(void)
{
    return &g_imu_data;
}

Motor_Control_t* Motor_GetData(void)
{
    return &g_motor_control;
}

// IMU数据处理函数
void ProcessIMUData(void)
{
    U8 rxByte;
    int processed_count = 0;
    const int MAX_PROCESS_PER_LOOP = 20;  // 限制每次处理的数据量
    
    // 一次性读取多个字节，减少中断操作
    while (UartFifo.Cnt > 0 && processed_count < MAX_PROCESS_PER_LOOP)
    {
        // 原子操作：一次性读取数据和更新指针
        __disable_irq();
        if (UartFifo.Cnt > 0) {
            rxByte = UartFifo.RxBuf[UartFifo.Out];
            if (++UartFifo.Out >= FifoSize) {
                UartFifo.Out = 0;
            }
            --UartFifo.Cnt;
        }
        __enable_irq();
        
        if (Cmd_GetPkt(rxByte)) {
            break;
        }
        processed_count++;
    }
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

// 初始化IMU
void imuInit(void)
{
    Cmd_03();// 1 唤醒IMU
    Cmd_12(5, 255, 0,  0, 3, 2, 2, 4, 9, 0xFFF);// 2 设置设备参数(内容1)
    Cmd_19();// 开启数据主动上报
}
