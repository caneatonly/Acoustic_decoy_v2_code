#ifndef __BSP_IO_INTERNAL_H
#define __BSP_IO_INTERNAL_H

#include "bsp_io.h"

// Expose writable globals only within peripherals implementation
extern MS5837_Data_t g_ms5837_data;
extern IMU_Data_t g_imu_data;

#endif /* __BSP_IO_INTERNAL_H */
