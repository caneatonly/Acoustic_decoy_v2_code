#include "bsp_usart.h"
#include "im948_CMD.h"

#include "stdio.h"
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

//------------------------------------------------------------------------------
// 描述: 被Cmd_Write调用，用于向IMU发送数据
// 返回: 返回发送字节数
//------------------------------------------------------------------------------
int UART_Write(uint8_t *buf, int Len)
{
	HAL_UART_Transmit(&huart2, buf, Len, 1000);
	return Len;
}



extern unsigned char Cmd_GetPkt(unsigned char byte);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  if (huart->Instance == USART2) 
		{
				Fifo_in(rx_byte);
				// 重新启用接收中断，以便继续接收数据
				HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

