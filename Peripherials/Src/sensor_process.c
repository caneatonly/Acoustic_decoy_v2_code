#include "sensor_process.h"
#include "bsp_usart.h"
#include "im948_CMD.h"

// IMU数据处理函数
void ProcessIMUData(void)
{
    U8 rxByte;
    int processed_count = 0;
    const int MAX_PROCESS_PER_LOOP = 20;  // 限制每次处理的数据量
    
    // 一次性读取多个字节，减少中断操作
    while (UartFifo.Cnt > 0 && processed_count < MAX_PROCESS_PER_LOOP)
    {
        // 原子操作：一次性读取数据和更新指针
        __disable_irq();
        if (UartFifo.Cnt > 0) {
            rxByte = UartFifo.RxBuf[UartFifo.Out];
            if (++UartFifo.Out >= FifoSize) {
                UartFifo.Out = 0;
            }
            --UartFifo.Cnt;
        }
        __enable_irq();
        
        if (Cmd_GetPkt(rxByte)) {
            break;
        }
        processed_count++;
    }
}


