#ifndef __BSP_USART_H

#define __BSP_USART_H

#include "main.h"
#include "usart.h"
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



int UART_Write(uint8_t *buf, int Len);



// UART1蓝牙调试数据处理函数
void UART1_DataHandler(void); //UART1蓝牙调试数据处理函数，在主循环中调用
void ProcessUART1Command(uint8_t *command, uint8_t length); //处理UART1接收到的命令 - 自定义接口在这里






#endif

