#ifndef __BSP_USART_H

#define __BSP_USART_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>



extern uint8_t rx_byte;
extern uint8_t uart3_rx_buffer[32]; 
extern uint8_t uart3_rx_byte;           // 单字节接收变量
extern uint8_t uart3_rx_index;          // 缓冲区索引
extern float temperature, depth;

int UART_Write(uint8_t *buf, int Len);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);



void ProcessUart3Data(uint8_t *data, uint8_t length);


#endif

