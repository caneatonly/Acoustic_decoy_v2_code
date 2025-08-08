#include "baro_adc.h"

// 内部参数与状态
#define BARO_ADC_BUF_SIZE 8

typedef struct {
    float vref;       // 参考电压，默认3.3V
    float v_min;      // 传感器有效下限，如0.2V
    float v_max;      // 传感器有效上限，如2.7V
    float p_min;      // 对应最小压力(kPa)
    float p_max;      // 对应最大压力(kPa)
    float p_offset;   // 校准偏移(kPa)
    float p_scale;    // 校准比例

    uint16_t buf[BARO_ADC_BUF_SIZE];
    uint8_t  idx;
    uint8_t  cnt;

    ADC_HandleTypeDef* hadc;
} BaroADC_Priv_t;

static BaroADC_Priv_t g_baro_priv = {
    .vref = 3.3f,
    .v_min = 0.2f,
    .v_max = 2.7f,
    .p_min = 0.0f,
    .p_max = 1500.0f,
    .p_offset = 0.0f,
    .p_scale  = 1.0f,
    .buf = {0},
    .idx = 0,
    .cnt = 0,
    .hadc = NULL,
};
 
static BaroADC_Data_t g_baro_data = {0};

static inline float clampf(float x, float a, float b){ return (x < a) ? a : (x > b) ? b : x; }

static uint16_t avg_u16(const uint16_t* a, uint8_t n){
    uint32_t s = 0; for(uint8_t i=0;i<n;i++) s += a[i]; return (uint16_t)(s / n);
}

static void baroadc_update_from_raw(uint16_t raw)
{
    // 原始→电压
    float voltage = (raw * g_baro_priv.vref) / 4095.0f; // 12-bit 满量程

    // 线性映射到压力（含夹紧）
    float v_clamped = clampf(voltage, g_baro_priv.v_min, g_baro_priv.v_max);
    float p_span = (g_baro_priv.p_max - g_baro_priv.p_min);
    float v_span = (g_baro_priv.v_max - g_baro_priv.v_min);
    float p = g_baro_priv.p_min + (p_span) * (v_clamped - g_baro_priv.v_min) / (v_span > 1e-6f ? v_span : 1e-6f);

    // 应用校准
    float p_cal = g_baro_priv.p_offset + p * g_baro_priv.p_scale;
    p_cal = clampf(p_cal, g_baro_priv.p_min, g_baro_priv.p_max);

    // 有效性：允许电压有±0.05V容差
    bool valid = (voltage >= (g_baro_priv.v_min - 0.05f)) && (voltage <= (g_baro_priv.v_max + 0.05f));

    g_baro_data.raw = raw;
    g_baro_data.voltage_v = voltage;
    g_baro_data.pressure_kpa = p_cal;
    g_baro_data.timestamp = HAL_GetTick();
    g_baro_data.data_valid = valid;
}

void BaroADC_Init(ADC_HandleTypeDef* hadc)
{
    g_baro_priv.hadc = hadc;
    g_baro_data.data_valid = false;
    g_baro_data.timestamp = 0;

    // 启动外部触发+中断（TIM3 TRGO 已在 Cube 配置）
    HAL_ADC_Start_IT(g_baro_priv.hadc);
}

void BaroADC_SetVref(float vref)
{
    if (vref > 1.0f && vref < 5.5f) g_baro_priv.vref = vref;
}

// 设置电压-压力映射
void BaroADC_SetMap(float v_min, float v_max, float p_min_kpa, float p_max_kpa)
{
    if (v_max > v_min + 0.1f) { g_baro_priv.v_min = v_min; g_baro_priv.v_max = v_max; }
    if (p_max_kpa > p_min_kpa) { g_baro_priv.p_min = p_min_kpa; g_baro_priv.p_max = p_max_kpa; }
}

// 设置压力校准
void BaroADC_SetCal(float p_offset_kpa, float p_scale)
{
    g_baro_priv.p_offset = p_offset_kpa;
    if (p_scale > 0.1f && p_scale < 10.0f) g_baro_priv.p_scale = p_scale;
}

BaroADC_Data_t* BaroADC_GetData(void)
{
    return &g_baro_data;
}

// HAL 回调：ADC 转换完成
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1 && g_baro_priv.hadc == hadc)
    {
        uint16_t val = (uint16_t)HAL_ADC_GetValue(hadc);

        // 环形缓冲记录并做平均
        g_baro_priv.buf[g_baro_priv.idx++] = val;
        if (g_baro_priv.idx >= BARO_ADC_BUF_SIZE) g_baro_priv.idx = 0;
        if (g_baro_priv.cnt < BARO_ADC_BUF_SIZE) g_baro_priv.cnt++;

        uint16_t filtered = (g_baro_priv.cnt == BARO_ADC_BUF_SIZE)
                          ? avg_u16(g_baro_priv.buf, BARO_ADC_BUF_SIZE)
                          : val;

        baroadc_update_from_raw(filtered);
    }
}
