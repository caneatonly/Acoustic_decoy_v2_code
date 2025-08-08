#ifndef __BARO_ADC_H
#define __BARO_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "adc.h"

// 气压传感器（模拟量）数据结构
typedef struct {
    float pressure_kpa;   // 计算后的压力值 (kPa)
    float voltage_v;      // 当前电压值 (V)
    uint16_t raw;         // 原始ADC值 (0-4095)
    uint32_t timestamp;   // 更新时间戳 (ms)
    bool data_valid;      // 数据有效性
} BaroADC_Data_t;

// 初始化与参数设置
void BaroADC_Init(ADC_HandleTypeDef* hadc);
void BaroADC_SetVref(float vref);
void BaroADC_SetMap(float v_min, float v_max, float p_min_kpa, float p_max_kpa);
void BaroADC_SetCal(float p_offset_kpa, float p_scale);

// 数据访问
BaroADC_Data_t* BaroADC_GetData(void);

#endif /* __BARO_ADC_H */
