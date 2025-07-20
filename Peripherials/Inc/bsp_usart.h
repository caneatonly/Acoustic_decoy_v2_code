#ifndef __BSP_USART_H

#define __BSP_USART_H

#include "main.h"
#include "usart.h"




extern uint8_t rx_byte;

int UART_Write(uint8_t *buf, int Len);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);




#endif

