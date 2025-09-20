#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

// Forward-declare sensor structs to decouple Control layer from peripheral headers
// We still need types' full definitions for field access; expose read-only pointers only.

typedef struct {
    float pressure_bag;
    float voltage_v;
    uint16_t raw;
    uint32_t timestamp;
    bool data_valid;
} BaroADC_Data_t;

typedef struct {
    float temperature;
    float depth;
    float pressure_water;
    uint32_t timestamp;
    bool data_valid;
} MS5837_Data_t;

typedef struct {
    float angleX;
    float angleY;
    float angleZ;
    float accelX;
    float accelY;
    float accelZ;
    uint32_t timestamp;
    bool data_valid;
} IMU_Data_t;

// 封装传感器数据访问接口
const BaroADC_Data_t* Ballon_pressure_get(void);
const MS5837_Data_t* MS5837_data_get(void);
const IMU_Data_t* IMU_data_get(void);

#ifdef __cplusplus
}
#endif
