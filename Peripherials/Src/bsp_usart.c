#include "bsp_usart.h"
#include "im948_CMD.h"
#include "sensor_process.h"
#include <stdlib.h>
#include <string.h>
#include "stdio.h"
#include "baro_adc.h"
#include "valve_ctrl.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

uint8_t rx_byte_debug;
uint8_t rx_byte;
uint8_t uart3_rx_buffer[32];
uint8_t uart3_rx_byte;
uint8_t uart3_rx_index=0;

// UART1蓝牙调试接收缓冲区
uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
volatile uint8_t uart1_rx_index = 0;
volatile uint8_t uart1_data_ready = 0;



/**
 * @brief UART1蓝牙调试数据处理函数，在主循环中调用
 */
void UART1_DataHandler(void)
{
    if (uart1_data_ready)
    {
        uart1_data_ready = 0;  // 清除数据准备标志
        
        // 处理接收到的命令
        ProcessUART1Command(uart1_rx_buffer, uart1_rx_index);
        
        // 重置缓冲区索引
        uart1_rx_index = 0;
    }
}

/**
 * @brief 处理UART1接收到的命令 - 自定义接口在这里
 * @param command 接收到的命令数据
 * @param length 命令长度
 */
void ProcessUART1Command(uint8_t *command, uint8_t length)
{
    // 添加字符串结束符
    command[length] = '\0';
    
    // 调试信息：显示接收到的命令
    printf("UART1 received [%d bytes]: %s\r\n", length, (char*)command);
    
    // 命令处理模板 - 用户可根据需要扩展
    if (strncmp((char*)command, "fairing", 7) == 0)
    {
        // 整流罩控制命令
        fairing_release();
        printf("Fairing release command executed\r\n");
    }
    else if (strncmp((char*)command, "valve_open", 10) == 0)
    {
        // 电磁阀开启命令
        valve_open();
        printf("Valve open command executed\r\n");
    }
    else if (strncmp((char*)command, "valve_close", 11) == 0)
    {
        // 电磁阀关闭命令
        valve_close();
        printf("Valve close command executed\r\n");
    }
    else if (strncmp((char*)command,"motortest", 9) == 0)
    {
        // 电机测试命令
        motor_test();
        printf("Motor test command executed\r\n");
    }
    else if (strncmp((char*)command, "status", 6) == 0)
    {
    IMU_Data_t* imu = IMU_GetData();
    const MS5837_Data_t* ms5837 = MS5837_GetData();
    const BaroADC_Data_t* baro = BaroADC_GetData();
        // 状态查询命令
        printf("System Status:\r\n");
        printf("  IMU Valid: %s\r\n", IMU_GetData()->data_valid ? "Yes" : "No");
    printf("  MS5837 Valid: %s\r\n", MS5837_GetData()->data_valid ? "Yes" : "No");
        printf("  BARO(ADC) Valid: %s\r\n", baro->data_valid ? "Yes" : "No");
        printf("Angle[%.2f,%.2f,%.2f] Accel[%.2f,%.2f,%.2f] | MS5837: T=%.2f D=%.2fm P=%.2fkPa | BARO: %.2fkPa (%.3fV, raw=%u)\r\n", 
            imu->angleX, imu->angleY, imu->angleZ,
            imu->accelX, imu->accelY, imu->accelZ,
            ms5837->temperature, ms5837->depth, ms5837->pressure_water, 
            baro->pressure_bag, baro->voltage_v, baro->raw);

    }
    else if (strncmp((char*)command, "reset", 5) == 0)
    {
        // 重置命令
        printf("System reset command received\r\n");
        NVIC_SystemReset();  // 执行系统重置
    }
    else if (strncmp((char*)command, "power_on", 8) == 0)
    {
        // 开机命令
        power_on();
        LEDstatus_on();  // 打开状态灯
        printf("Power on command received\r\n");
        // Add code to handle power on functionality here
    }
    else if (strncmp((char*)command, "power_off", 9) == 0)
    {
        // 关机命令
        power_off();
        LEDstatus_off();  // 关闭状态灯
        printf("Power off command received\r\n");
        // Add code to handle power off functionality here
    }
    else if (strncmp((char*)command, "set kp ", 7) == 0)
    {
        float v;
        if (sscanf((char*)command+7, "%f", &v) == 1)
        {
            Valve_ControlAlgorithm_SetGains(v, -1.0f);
            printf("Kp set to %.4f\r\n", v);
        }
        else
        {
            printf("Usage: set kp <value>\r\n");
        }
    }
    else if (strncmp((char*)command, "set kd ", 7) == 0)
    {
        float v;
        if (sscanf((char*)command+7, "%f", &v) == 1)
        {
            Valve_ControlAlgorithm_SetGains(-1.0f, v);
            printf("Kd set to %.4f\r\n", v);
        }
        else
        {
            printf("Usage: set kd <value>\r\n");
        }
    }
    else if (strncmp((char*)command, "set margin ", 11) == 0)
    {
        float v;
        if (sscanf((char*)command+11, "%f", &v) == 1)
        {
            Valve_ControlAlgorithm_SetMargin(v);
            printf("Margin set to %.3f kPa\r\n", v);
        }
        else
        {
            printf("Usage: set margin <kPa>\r\n");
        }
    }
    else if (strncmp((char*)command, "set eps ", 8) == 0)
    {
        float v;
        if (sscanf((char*)command+8, "%f", &v) == 1)
        {
            Valve_ControlAlgorithm_SetEps(v);
            printf("Eps set to %.3f kPa\r\n", v);
        }
        else
        {
            printf("Usage: set eps <kPa>\r\n");
        }
    }
    else if (strncmp((char*)command, "set guard ", 10) == 0)
    {
        float v;
        if (sscanf((char*)command+10, "%f", &v) == 1)
        {
            Valve_ControlAlgorithm_SetGuard(v);
            printf("Guard set to %.1f kPa\r\n", v);
        }
        else
        {
            printf("Usage: set guard <kPa>\r\n");
        }
    }
    else if (strncmp((char*)command, "set window ", 11) == 0)
    {
        unsigned on_ms, off_ms;
        if (sscanf((char*)command+11, "%u %u", &on_ms, &off_ms) == 2)
        {
            Valve_ControlAlgorithm_SetWindow(on_ms, off_ms);
            printf("Window min_on=%u ms, min_off=%u ms\r\n", on_ms, off_ms);
        }
        else
        {
            printf("Usage: set window <min_on_ms> <min_off_ms>\r\n");
        }
    }
    else if (strncmp((char*)command, "params", 6) == 0)
    {
        ValveControlParams_t p; Valve_ControlAlgorithm_GetParams(&p);
        printf("Params: Kp=%.4f Kd=%.4f eps=%.3f kPa margin=%.3f kPa guard=%.1f kPa window=%u ms min_on=%u ms min_off=%u ms\r\n",
            p.Kp, p.Kd, p.eps_kpa, p.dp_margin_kpa, p.guard_over_kpa, p.window_ms, p.Tmin_on_ms, p.Tmin_off_ms);
    }
    else
    {
        // 未知命令
        printf("Unknown command: %s\r\n", (char*)command);
        printf("Available commands: \r\n");
        printf("motortest, fairing, valve_open, valve_close, status, reset\r\n");
        printf("set kp <v>, set kd <v>, set margin <kPa>, set eps <kPa>, set guard <kPa>, set window <on off>, params\r\n");
    }
}

// 描述: 被Cmd_Write调用，用于向IMU发送数据
// 返回: 返回发送字节数
int UART_Write(uint8_t *buf, int Len)
{
	HAL_UART_Transmit(&huart2, buf, Len, 1000);
	return Len;
}


/**
    * @brief UART接收完成回调函数
    * @param huart 指向UART句柄的指针
    * 
    * 处理不同UART实例的接收数据：
    * - USART1: 蓝牙调试命令接收与状态发送
    * - USART2: IMU数据接收
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
    if (huart->Instance == USART1)
    {
        // 快速处理：将接收数据存入缓冲区，设置标志位
        if (uart1_rx_index < (UART1_RX_BUFFER_SIZE - 1))
        {
            uart1_rx_buffer[uart1_rx_index++] = rx_byte_debug;
            
            // 检测命令结束条件：感叹号
            if (rx_byte_debug == '1')
            {
                if (uart1_rx_index > 1)  // 确保有有效数据
                {
                    uart1_rx_index--;  // 移除结束符
                    uart1_data_ready = 1;  // 设置数据准备标志
                }
                else
                {
                    uart1_rx_index = 0;  // 重置索引
                }
            }
        }
        else
        {
            // 缓冲区溢出，重置
            uart1_rx_index = 0;
        }
        // 重新启用接收中断
        HAL_UART_Receive_IT(huart, &rx_byte_debug, 1);
    }
    else if (huart->Instance == USART2)
    {
        Fifo_in(rx_byte);
        // 重新启用接收中断，以便继续接收数据
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
    // else if (huart->Instance == USART3)
    // {
    //     // 防止缓冲区溢出
    //     if (uart3_rx_index >= 31)
    //     {
    //         uart3_rx_index = 0;  // 重置缓冲区
    //     }
        
    //     // 将接收到的字节存入缓冲区
    //     uart3_rx_buffer[uart3_rx_index++] = uart3_rx_byte;
        
    //     // 检测数据包结束条件：换行符、回车符
    //     if (uart3_rx_byte == '\n' || uart3_rx_byte == '\r')
    //     {
    //         // 添加字符串结束符
    //         uart3_rx_buffer[uart3_rx_index] = '\0';
            
    //         // 只有当接收到有效数据时才处理
    //         if (uart3_rx_index > 1)
    //         {
    //             ProcessUart3Data(uart3_rx_buffer);
    //         }
            
    //         // 重置索引，准备接收下一个数据包
    //         uart3_rx_index = 0;
    //     }
        
    //     // 重新启动单字节接收
    //     HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    // }
}

