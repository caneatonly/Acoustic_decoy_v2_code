#include "bsp_usart.h"
#include "console.h"
#include "im948_CMD.h"
#include "bsp_io.h"
#include "control_tasks.h"
#include "control_config.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "stdio.h"
#include "baro_adc.h"
#include "valve_ctrl.h"
#include "FreeRTOS.h"
#include "task.h"

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
uint8_t uart3_rx_buffer[32];
uint8_t uart3_rx_byte;
uint8_t uart3_rx_index=0;

// UART1蓝牙调试接收缓冲区
uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
volatile uint8_t uart1_rx_index = 0;
volatile uint8_t uart1_data_ready = 0;
static volatile uint8_t uart1_overflow = 0;

static volatile uint32_t uart1_error_count = 0;
static volatile uint32_t uart2_error_count = 0;
static volatile uint32_t uart3_error_count = 0;

#define IMU_UART_DMA_BUFFER_COUNT   2U
#define IMU_UART_DMA_BUFFER_SIZE    256U

static uint8_t imu_dma_buffer[IMU_UART_DMA_BUFFER_COUNT][IMU_UART_DMA_BUFFER_SIZE];
static uint8_t imu_dma_active_index = 0U;
static uint16_t imu_dma_consumed = 0U;

// Simple version string for 'ver' command
static const char* FW_VERSION_STR = "Acoustic_decoy_v2 FW - based on FreeRTOS " __DATE__ " " __TIME__;



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
            console_printf("ERR: line too long (max %d)\r\n", UART1_RX_BUFFER_SIZE - 1);
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
    console_printf("UART1 received [%u bytes]: %s\r\n", (unsigned)n, cmd);
    
    // 命令处理 - 可扩展
    if (n == 0)
    {
        return;
    }
    else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    {
        console_printf("=== Acoustic Decoy v2 Control Commands ===\r\n");
        console_printf("\r\n");
        console_printf("System Control:\r\n");
        console_printf("  ctrl on|off|?         - Enable/disable auto control\r\n");
        console_printf("                          ON:  Mission FSM active (auto mode)\r\n");
        console_printf("                          OFF: Manual control mode\r\n");
        console_printf("                          ?:   Query current state\r\n");
        console_printf("\r\n");
        console_printf("Manual Actuators (requires 'ctrl off'):\r\n");
        console_printf("  motortest             - Test motor (PWM 2000->1500, 1s pulse)\r\n");
        console_printf("  valve_open            - Open inflation valve\r\n");
        console_printf("  valve_close           - Close inflation valve\r\n");
        console_printf("  fairing               - Release nose fairing (100ms pulse)\r\n");
        console_printf("\r\n");
        console_printf("System Info:\r\n");
        console_printf("  status                - Show sensor status\r\n");
        console_printf("  ver                   - Firmware version\r\n");
        console_printf("\r\n");
        console_printf("Power Control:\r\n");
        console_printf("  power_on              - Enable 12V power\r\n");
        console_printf("  power_off             - Disable 12V power\r\n");
        console_printf("  reset                 - System reset\r\n");
        console_printf("\r\n");
    }
    else if (strcmp(cmd, "ver") == 0)
    {
        console_printf("%s\r\n", FW_VERSION_STR);
    }
    else if (strcmp(cmd, "fairing") == 0)
    {
        // 整流罩控制命令
        fairing_release();
        console_printf("Fairing release command executed\r\n");
    }
    else if (strcmp(cmd, "valve_open") == 0)
    {
        // 电磁阀开启命令
        valve_open();
        console_printf("Valve open command executed\r\n");
    }
    else if (strcmp(cmd, "valve_close") == 0)
    {
        // 电磁阀关闭命令
        valve_close();
        console_printf("Valve close command executed\r\n");
    }
    else if (strcmp(cmd,"motortest") == 0)
    {
        // 电机测试命令
        motor_test();
        console_printf("Motor test command executed\r\n");
    }
    else if (strcmp(cmd, "status") == 0)
    {
    const IMU_Data_t* imu = IMU_GetData();
    const MS5837_Data_t* ms5837 = MS5837_GetData();
    const BaroADC_Data_t* baro = BaroADC_GetData();
        // 状态查询命令
        console_printf("System Status:\r\n");
        console_printf("  IMU Valid: %s\r\n", IMU_GetData()->data_valid ? "Yes" : "No");
        console_printf("  MS5837 Valid: %s\r\n", MS5837_GetData()->data_valid ? "Yes" : "No");
        console_printf("  BARO(ADC) Valid: %s\r\n", baro->data_valid ? "Yes" : "No");
        console_printf("Angle[%.2f,%.2f,%.2f] Accel[%.2f,%.2f,%.2f] | MS5837: T=%.2f D=%.2fm P=%.2fkPa | BARO: %.2fkPa (%.3fV, raw=%u)\r\n", 
            imu->angleX, imu->angleY, imu->angleZ,
            imu->accelX, imu->accelY, imu->accelZ,
            ms5837->temperature, ms5837->depth, ms5837->pressure_water, 
            baro->pressure_bag, baro->voltage_v, baro->raw);

    }
    else if (strcmp(cmd, "reset") == 0)
    {
        // 重置命令
        console_printf("System reset command received\r\n");
        NVIC_SystemReset();  // 执行系统重置
    }
    else if (strcmp(cmd, "power_on") == 0)
    {
        // 12V 电源控制
        power_on();
        LEDstatus_on();  // 打开状态灯
        console_printf("Power on command received\r\n");
        // Add code to handle power on functionality here
    }
    else if (strcmp(cmd, "power_off") == 0)
    {
        // 12V 电源控制
        power_off();
        LEDstatus_off();  // 关闭状态灯
        console_printf("Power off command received\r\n");
        // Add code to handle power off functionality here
    }
    else if (strncmp(cmd, "ctrl ", 5) == 0)
    {
        extern volatile uint8_t g_control_loop_enabled; // defined in main.c
        if (strcmp(cmd+5, "on") == 0) { 
            g_control_loop_enabled = 1; 
            console_printf("ctrl: ON - Auto control resumed (Mission FSM active)\r\n"); 
        }
        else if (strcmp(cmd+5, "off") == 0) { 
            g_control_loop_enabled = 0;
            
            // 安全清理：禁用控制循环时，设置电机为中性，关闭阀门
            SetMotorSpeed(CTRL_PWM_NEUTRAL); // 1500 中性位置
            valve_close();                   // 关闭充气阀
            
            console_printf("ctrl: OFF - Manual control mode\r\n");
            console_printf("  Motor: PWM=%d (neutral)\r\n", CTRL_PWM_NEUTRAL);
            console_printf("  Valve: CLOSED\r\n");
            console_printf("  Now you can use manual commands: motortest, valve_open, valve_close, etc.\r\n");
        }
        else if (strcmp(cmd+5, "?") == 0) { 
            console_printf("ctrl: %s\r\n", g_control_loop_enabled?"ON (Auto)":"OFF (Manual)"); 
        }
        else { 
            console_printf("ERR: usage ctrl on|off|?\r\n"); 
        }
    }
    else
    {
        // 未知命令
        console_printf("Unknown command: %s\r\n", cmd);
        console_printf("Type 'help' for a list of commands.\r\n");
    }
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
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint8_t *rx_buf = imu_dma_buffer[imu_dma_active_index];

        if ((g_imuRxQueue != NULL) && (Size > imu_dma_consumed) && (Size <= IMU_UART_DMA_BUFFER_SIZE))
        {
            uint16_t start = imu_dma_consumed;
            uint16_t new_bytes = Size - imu_dma_consumed;

            ImuRxBlock_t block = {
                .data = &rx_buf[start],
                .length = new_bytes
            };

            if (xQueueSendFromISR(g_imuRxQueue, &block, &xHigherPriorityTaskWoken) == pdPASS)
            {
                imu_dma_consumed = Size;
            }
            else
            {
                uart2_error_count++;
            }
        }

        HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);
        if ((event == HAL_UART_RXEVENT_IDLE) || (event == HAL_UART_RXEVENT_TC))
        {
            uint8_t next_index = (imu_dma_active_index + 1U) % IMU_UART_DMA_BUFFER_COUNT;
            imu_dma_consumed = 0U;

            if (HAL_UARTEx_ReceiveToIdle_DMA(huart, imu_dma_buffer[next_index], IMU_UART_DMA_BUFFER_SIZE) == HAL_OK)
            {
                imu_dma_active_index = next_index;
            }
            else
            {
                uart2_error_count++;
                /* 再次尝试使用当前缓冲区恢复接收 */
                (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, imu_dma_buffer[imu_dma_active_index], IMU_UART_DMA_BUFFER_SIZE);
            }
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_error_count++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(huart, &rx_byte_debug, 1);
    }
    else if (huart->Instance == USART2)
    {
        uart2_error_count++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_DMAStop(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        imu_dma_active_index = 0U;
        imu_dma_consumed = 0U;
        (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, imu_dma_buffer[imu_dma_active_index], IMU_UART_DMA_BUFFER_SIZE);
    }
    else if (huart->Instance == USART3)
    {
        uart3_error_count++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_Receive_IT(huart, &uart3_rx_byte, 1);
    }
}

HAL_StatusTypeDef IMU_UART_StartDmaReception(void)
{
    imu_dma_active_index = 0U;
    imu_dma_consumed = 0U;
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart2, imu_dma_buffer[imu_dma_active_index], IMU_UART_DMA_BUFFER_SIZE);
}

