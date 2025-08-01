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

uint8_t rx_byte_debug;
uint8_t rx_byte;
uint8_t uart3_rx_buffer[32];
uint8_t uart3_rx_byte;
uint8_t uart3_rx_index=0;

// 蓝牙连接状态变量
volatile uint8_t bt_connected = 0;        // 蓝牙连接状态：0=未连接，1=已连接
volatile uint8_t bt_status_changed = 0;   // 状态变化标志：0=无变化，1=有变化
volatile uint32_t bt_debounce_time = 0;   // 防抖时间戳

#define BT_DEBOUNCE_DELAY_MS 50           // 防抖延时50ms

/**
 * @brief 蓝牙状态初始化
 */
void BT_StatusInit(void)
{
    // 读取当前PC4引脚状态
    bt_connected = HAL_GPIO_ReadPin(BT_status_GPIO_Port, BT_status_Pin);
    bt_status_changed = 0;
    
    printf("BT Status initialized: %s\r\n", bt_connected ? "Connected" : "Disconnected");
}

/**
 * @brief 蓝牙状态处理函数，在主循环中调用
 */
void BT_StatusHandler(void)
{
    // 处理防抖逻辑
    if (bt_debounce_time != 0)
    {
        uint32_t current_time = HAL_GetTick();
        if (current_time - bt_debounce_time >= BT_DEBOUNCE_DELAY_MS)
        {
            // 防抖时间到，重新读取引脚状态确认
            uint8_t current_status = HAL_GPIO_ReadPin(BT_status_GPIO_Port, BT_status_Pin);
            
            if (current_status != bt_connected)
            {
                bt_connected = current_status;
                bt_status_changed = 1;  // 设置状态变化标志
            }
            
            bt_debounce_time = 0;  // 清除防抖时间戳
        }
    }
    
    // 处理状态变化
    if (bt_status_changed)
    {
        bt_status_changed = 0;  // 清除状态变化标志
        
        if (bt_connected)
        {
            // 蓝牙连接时发送上线消息
            printf("Acoustic_decoy is online.\r\n");
        }
    }
}

/**
 * @brief GPIO外部中断回调函数
 * @param GPIO_Pin 触发中断的GPIO引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BT_status_Pin)
    {
        // 快速处理：仅设置防抖时间戳，实际状态检测在主循环中完成
        bt_debounce_time = HAL_GetTick();
    }
}

// 描述: 被Cmd_Write调用，用于向IMU发送数据
// 返回: 返回发送字节数
int UART_Write(uint8_t *buf, int Len)
{
	HAL_UART_Transmit(&huart2, buf, Len, 1000);
	return Len;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
    if (huart->Instance == USART1)
    {
        printf("USART1 received: 0x%02X ('%c')\r\n", rx_byte_debug, rx_byte_debug);
        HAL_UART_Receive_IT(huart, &rx_byte_debug, 1);
        
        // // 判断接收到的字节是否为'1'
        // if (rx_byte_debug == '1')
        // {
        //     fairing_release();
        //     printf("faring release command received\r\n");
        // }
    }
    else if (huart->Instance == USART2)
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
                ProcessUart3Data(uart3_rx_buffer);
            }
            
            // 重置索引，准备接收下一个数据包
            uart3_rx_index = 0;
        }
        
        // 重新启动单字节接收
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

