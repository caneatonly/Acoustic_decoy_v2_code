#include "sensor_process.h"
#include "bsp_usart.h"
#include "im948_CMD.h"
#include "stm32f1xx_hal.h"
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
    const int MAX_PROCESS_PER_LOOP = 200;  // 限制每次处理的数据量
    
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

// 深度传感器数据处理
void ProcessUart3Data(uint8_t *data) {
    // 手动解析
    char *t_pos = strstr((char *)data, "T=");
    char *d_pos = strstr((char *)data, "D=");
    if (t_pos && d_pos) {
      float temp = atof(t_pos + 2);
      float depth = atof(d_pos + 2);
    // 检查数据范围
      if (temp >= -40.0f && temp <= 85.0f && depth >= -10.0f && depth <= 300.0f) {
        g_ms5837_data.temperature = temp;
        g_ms5837_data.depth = depth;
        g_ms5837_data.timestamp = HAL_GetTick();
        g_ms5837_data.data_valid = true;
        
        // 1秒输出一次
        static uint32_t last_print_time = 0;
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_print_time >= 1000) {
            Dbp("MS5837 - T:%.2f°C, D:%.2fm\r\n", g_ms5837_data.temperature, g_ms5837_data.depth);
            last_print_time = current_time;
        }
      } else {
        g_ms5837_data.data_valid = false;
        Dbp("MS5837 data out of range\r\n");
      }
    } else {
      g_ms5837_data.data_valid = false;
      Dbp("MS5837 parse failed: %s\r\n", data);
    }
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

void fairing_release(void){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
    HAL_Delay(100); // 等待100ms以确保释放完成
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
}
void fairing_retract(void){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
}

void valve_open(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
}
void valve_close(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
}
