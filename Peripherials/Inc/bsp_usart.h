#ifndef __BSP_USART_H

#define __BSP_USART_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "sensor_process.h"

extern uint8_t rx_byte_debug;
extern uint8_t rx_byte;
extern uint8_t uart3_rx_buffer[32]; 
extern uint8_t uart3_rx_byte;           // 单字节接收变量
extern uint8_t uart3_rx_index;          // 缓冲区索引

// UART1蓝牙调试接收缓冲区
#define UART1_RX_BUFFER_SIZE 64
extern uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
extern volatile uint8_t uart1_rx_index;
extern volatile uint8_t uart1_data_ready;    // 数据准备标志


// 蓝牙连接状态变量
extern volatile uint8_t bt_connected;        // 蓝牙连接状态标志
extern volatile uint8_t bt_status_changed;   // 蓝牙状态变化标志
extern volatile uint32_t bt_debounce_time;   // 防抖时间戳

int UART_Write(uint8_t *buf, int Len);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 蓝牙状态处理函数
void BT_StatusInit(void);
void BT_StatusHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// UART1蓝牙调试数据处理函数
void UART1_DataHandler(void);
void ProcessUART1Command(uint8_t *command, uint8_t length);






#endif

