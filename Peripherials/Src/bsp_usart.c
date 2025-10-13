#include "bsp_usart.h"
#include "im948_CMD.h"
#include "bsp_io.h"
#include "control_tasks.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
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
static volatile uint8_t uart1_overflow = 0;

// Simple version string for 'ver' command
static const char* FW_VERSION_STR = "Acoustic_decoy_v2 FW - build " __DATE__ " " __TIME__;



/**
 * @brief UART1蓝牙调试数据处理函数，在主循环中调用
 */
void UART1_DataHandler(void)
{
    if (uart1_data_ready)
    {
        uart1_data_ready = 0;  // 清除数据准备标志
        // 结束符与溢出安全：补0并修剪尾部空白
        if (uart1_rx_index >= (UART1_RX_BUFFER_SIZE - 1)) {
            uart1_rx_index = (UART1_RX_BUFFER_SIZE - 1);
        }
        uart1_rx_buffer[uart1_rx_index] = '\0';

        // 去除行尾的 \r 和空白
        int len = (int)uart1_rx_index;
        while (len > 0 && (uart1_rx_buffer[len - 1] == '\r' || isspace((int)uart1_rx_buffer[len - 1]))) {
            uart1_rx_buffer[--len] = '\0';
        }

        // 跳过行首空白
        int start = 0;
        while (uart1_rx_buffer[start] != '\0' && isspace((int)uart1_rx_buffer[start])) {
            start++;
        }

        if (uart1_overflow) {
            printf("ERR: line too long (max %d)\r\n", UART1_RX_BUFFER_SIZE - 1);
            uart1_overflow = 0;
        } else if (uart1_rx_buffer[start] != '\0') {
            // 处理接收到的命令
            ProcessUART1Command(&uart1_rx_buffer[start], (uint8_t)strlen((char*)&uart1_rx_buffer[start]));
        }

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
    // 添加字符串结束符（保护）
    command[length] = '\0';

    // 左右修剪（再次确保干净）
    char *cmd = (char*)command;
    while (*cmd && isspace((int)*cmd)) cmd++;
    size_t n = strlen(cmd);
    while (n > 0 && isspace((int)cmd[n-1])) { cmd[--n] = '\0'; }

    // 调试信息：显示接收到的命令
    printf("UART1 received [%u bytes]: %s\r\n", (unsigned)n, cmd);
    
    // 命令处理 - 可扩展
    if (n == 0)
    {
        return;
    }
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    {
        printf("Commands:\r\n");
        printf("  help|?                - show this help\r\n");
        printf("  ver                   - firmware version\r\n");
        printf("  status                - quick system status\r\n");
        printf("  fairing               - release fairing\r\n");
        printf("  valve_open|valve_close - control valve\r\n");
        printf("  motortest             - motor test pulse\r\n");
        printf("  power_on|power_off    - 12V power control\r\n");
        printf("  reset                 - System reset\r\n");
        printf("  ctrl on|off|?         - enable/disable control loop, '?' shows state\r\n");
        printf("  set b.kp=<f> b.kd=<f> b.margin=<f> b.eps=<f>\r\n");
        printf("  params                - dump valve PD params\r\n");
    }
    else if (strcmp(cmd, "ver") == 0)
    {
        printf("%s\r\n", FW_VERSION_STR);
    }
    else if (strcmp(cmd, "fairing") == 0)
    {
        // 整流罩控制命令
        fairing_release();
        printf("Fairing release command executed\r\n");
    }
    else if (strcmp(cmd, "valve_open") == 0)
    {
        // 电磁阀开启命令
        valve_open();
        printf("Valve open command executed\r\n");
    }
    else if (strcmp(cmd, "valve_close") == 0)
    {
        // 电磁阀关闭命令
        valve_close();
        printf("Valve close command executed\r\n");
    }
    else if (strcmp(cmd,"motortest") == 0)
    {
        // 电机测试命令
        motor_test();
        printf("Motor test command executed\r\n");
    }
    else if (strcmp(cmd, "status") == 0)
    {
    const IMU_Data_t* imu = IMU_GetData();
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
    else if (strcmp(cmd, "reset") == 0)
    {
        // 重置命令
        printf("System reset command received\r\n");
        NVIC_SystemReset();  // 执行系统重置
    }
    else if (strcmp(cmd, "power_on") == 0)
    {
        // 12V 电源控制
        power_on();
        LEDstatus_on();  // 打开状态灯
        printf("Power on command received\r\n");
        // Add code to handle power on functionality here
    }
    else if (strcmp(cmd, "power_off") == 0)
    {
        // 12V 电源控制
        power_off();
        LEDstatus_off();  // 关闭状态灯
        printf("Power off command received\r\n");
        // Add code to handle power off functionality here
    }
    else if (strncmp(cmd, "set b.kp=", 10) == 0)
    {
        float v;
        if (sscanf(cmd+10, "%f", &v) == 1) { Valve_ControlAlgorithm_SetGains(v, -1.0f); printf("b.kp=%.4f OK\r\n", v);} 
        else { printf("ERR: usage set b.kp=<float>\r\n"); }
    }
    else if (strncmp(cmd, "set b.kd=", 10) == 0)
    {
        float v;
        if (sscanf(cmd+10, "%f", &v) == 1) { Valve_ControlAlgorithm_SetGains(-1.0f, v); printf("b.kd=%.4f OK\r\n", v);} 
        else { printf("ERR: usage set b.kd=<float>\r\n"); }
    }
    else if (strncmp(cmd, "set b.margin=", 13) == 0)
    {
        float v;
        if (sscanf(cmd+13, "%f", &v) == 1) { Valve_ControlAlgorithm_SetMargin(v); printf("b.margin=%.3f OK\r\n", v);} 
        else { printf("ERR: usage set b.margin=<kPa>\r\n"); }
    }
    else if (strncmp(cmd, "set b.eps=", 11) == 0)
    {
        float v;
        if (sscanf(cmd+11, "%f", &v) == 1) { Valve_ControlAlgorithm_SetEps(v); printf("b.eps=%.3f OK\r\n", v);} 
        else { printf("ERR: usage set b.eps=<kPa>\r\n"); }
    }
    else if (strncmp(cmd, "ctrl ", 5) == 0)
    {
        extern volatile uint8_t g_control_loop_enabled; // defined in main.c
        if (strcmp(cmd+5, "on") == 0) { g_control_loop_enabled = 1; printf("ctrl: ON\r\n"); }
        else if (strcmp(cmd+5, "off") == 0) { g_control_loop_enabled = 0; printf("ctrl: OFF\r\n"); }
        else if (strcmp(cmd+5, "?") == 0) { printf("ctrl: %s\r\n", g_control_loop_enabled?"ON":"OFF"); }
        else { printf("ERR: usage ctrl on|off|?\r\n"); }
    }
    else if (strncmp(cmd, "params", 6) == 0)
    {
        ValveControlParams_t p; Valve_ControlAlgorithm_GetParams(&p);
        printf("Params: b.kp=%.4f b.kd=%.4f b.eps=%.3f kPa b.margin=%.3f kPa\r\n",
            p.Kp, p.Kd, p.eps_kpa, p.dp_margin_kpa);
    }
    else
    {
        // 未知命令
        printf("Unknown command: %s\r\n", cmd);
        printf("Type 'help' for a list of commands.\r\n");
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
        //以 \n/\r 作为行结束
        const uint8_t ch = rx_byte_debug;
        const uint8_t BS = 0x08;         // backspace
        const uint8_t DEL = 0x7F;        // delete
        const uint8_t CR = '\r';
        const uint8_t LF = '\n';

        if (ch == BS || ch == DEL) {
            if (uart1_rx_index > 0) {
                uart1_rx_index--; // 简单本地编辑：仅索引回退
            }
        } else if (ch == CR || ch == LF) {
            // 忽略连续的 CR/LF 组合中的第二个
            if (uart1_rx_index > 0) {
                uart1_data_ready = 1;
            } else {
                // 空行，忽略
            }
        } else {
            if (uart1_rx_index < (UART1_RX_BUFFER_SIZE - 1)) {
                uart1_rx_buffer[uart1_rx_index++] = ch;
            } else {
                // 标记溢出并丢弃后续字符直到行结束
                uart1_overflow = 1;
            }
        }
        // 重新启用接收中断
        HAL_UART_Receive_IT(huart, &rx_byte_debug, 1);
    }
    else if (huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (g_imuRxQueue != NULL)
        {
            (void)xQueueSendFromISR(g_imuRxQueue, &rx_byte, &xHigherPriorityTaskWoken);
        }

        // 重新启用接收中断，以便继续接收数据
        HAL_UART_Receive_IT(huart, &rx_byte, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

}

