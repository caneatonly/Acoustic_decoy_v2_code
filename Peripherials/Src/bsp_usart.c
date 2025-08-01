#include "bsp_usart.h"
#include "im948_CMD.h"


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

// 描述: 被Cmd_Write调用，用于向IMU发送数据
// 返回: 返回发送字节数
int UART_Write(uint8_t *buf, int Len)
{
	HAL_UART_Transmit(&huart2, buf, Len, 1000);
	return Len;
}

// 深度传感器数据处理
void ProcessUart3Data(uint8_t *data, uint8_t length)
{
		
        // 尝试手动解析
        char *t_pos = strstr((char*)data, "T=");
        char *d_pos = strstr((char*)data, "D=");
        
        if (t_pos && d_pos) {
            temperature = atof(t_pos + 2);
            depth = atof(d_pos + 2);
            Dbp("Manual parse - Temperature: %.2f°C, Depth: %.2fm\r\n", temperature, depth);
        } else {
            Dbp("Parse failed. Raw data [%d bytes]: %s\r\n", length, data);
        }
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
    else if (huart->Instance == USART3)
    {
        // 防止缓冲区溢出
        if (uart3_rx_index >= 31)
        {
            uart3_rx_index = 0;  // 重置缓冲区
        }
        
        // 将接收到的字节存入缓冲区
        uart3_rx_buffer[uart3_rx_index++] = uart3_rx_byte;
        
        // 检测数据包结束条件：换行符、回车符
        if (uart3_rx_byte == '\n' || uart3_rx_byte == '\r')
        {
            // 添加字符串结束符
            uart3_rx_buffer[uart3_rx_index] = '\0';
            
            // 只有当接收到有效数据时才处理
            if (uart3_rx_index > 1)
            {
                ProcessUart3Data(uart3_rx_buffer, uart3_rx_index);
            }
            
            // 重置索引，准备接收下一个数据包
            uart3_rx_index = 0;
        }
        
        // 重新启动单字节接收
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

