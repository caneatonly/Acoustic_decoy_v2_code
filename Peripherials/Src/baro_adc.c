#include "baro_adc.h"

// 内部参数与状态
#define BARO_ADC_BUF_SIZE 8

typedef struct {
    float vref;       // 参考电压，默认3.3V
    float sensitivity; // 传感器灵敏度，默认0.00167 V/kPa
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
    .sensitivity = 0.00167f,
    .v_min = 0.2f,      // 传感器最小输出电压 (对应0压力)
    .v_max = 2.705f,    // 传感器最大输出电压 (对应1500kPa: 0.2 + 1500*0.00167)
    .p_min = 0.0f,
    .p_max = 1500.0f,
    .p_offset = 100.98f, // 默认大气压约101 kPa（后期可修改为初始化时从MS5837数据结构体读取）
    .p_scale  = 1.0f,
    .buf = {0},
    .idx = 0,
    .cnt = 0,
    .hadc = NULL,
};
 
static BaroADC_Data_t g_baro_data = {0};

static inline float clampf(float x, float a, float b){ return (x < a) ? a : (x > b) ? b : x; }

static uint16_t avg_u16(const uint16_t* a, uint8_t n){
    uint32_t s = 0; 
    for (uint8_t i = 0; i < n; i++) {
    s += a[i];
    }
return (uint16_t)(s / n);
}

static void baroadc_update_from_raw(uint16_t raw)
{
    // 原始→电压
    float voltage = (raw * g_baro_priv.vref) / 4095.0f; // 12-bit 满量程

    // 根据传感器公式计算压力：Vout = P*0.00167 + 0.2
    // 反推公式：P = (Vout - 0.2) / 0.00167
    float pressure_raw = 0.0f;
    
    if (voltage >= g_baro_priv.v_min) {
        pressure_raw = (voltage - g_baro_priv.v_min) / g_baro_priv.sensitivity; // kPa
    } else {
        pressure_raw = 0.0f; // 电压低于零点，压力为0
    }

    // 应用用户校准，加上大气压
    float p_cal = g_baro_priv.p_offset + pressure_raw * g_baro_priv.p_scale;
    
    // 限制在合理范围内
    p_cal = clampf(p_cal, 0.0f, g_baro_priv.p_max);

    // 有效性判断：电压应在传感器规格范围内
    // 根据公式，0.2V对应0压力，假设最大压力1500kPa对应2.705V
    float v_min_spec = g_baro_priv.v_min; // 0.2V
    float v_max_spec = g_baro_priv.v_min + g_baro_priv.p_max * g_baro_priv.sensitivity; // 计算最大电压
    bool valid = (voltage >= (v_min_spec - 0.05f)) && (voltage <= (v_max_spec + 0.05f));

    g_baro_data.raw = raw;
    g_baro_data.voltage_v = voltage;
    g_baro_data.pressure_bag = p_cal;
    g_baro_data.timestamp = HAL_GetTick();
    g_baro_data.data_valid = valid;
}

void BaroADC_Init(ADC_HandleTypeDef* hadc)
{
    g_baro_priv.hadc = hadc;
    g_baro_data.data_valid = false;
    g_baro_data.timestamp = 0;
    //校准
    HAL_ADCEx_Calibration_Start(hadc);
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
