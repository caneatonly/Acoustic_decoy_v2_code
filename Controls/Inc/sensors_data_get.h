#pragma once

// 传感器数据访问接口层
// 提供统一的传感器数据读取接口

#include "bsp_io.h"       // MS5837_Data_t, IMU_Data_t
#include "baro_adc.h"     // BaroADC_Data_t

#ifdef __cplusplus
extern "C" {
#endif

// 封装传感器数据访问接口
// 返回只读指针，指向外设层维护的数据结构
const BaroADC_Data_t* Ballon_pressure_get(void);
const MS5837_Data_t* MS5837_data_get(void);
const IMU_Data_t* IMU_data_get(void);

#ifdef __cplusplus
}
#endif
