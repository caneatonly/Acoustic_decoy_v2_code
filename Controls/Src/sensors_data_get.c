// This C file intentionally does NOT include sensors_data_get.h to avoid
// duplicate typedefs; it only bridges to peripheral providers.
#include "baro_adc.h"
#include "bsp_io.h"

const BaroADC_Data_t* Ballon_pressure_get(void) {
    return BaroADC_GetData();
}

const MS5837_Data_t* MS5837_data_get(void) {
    return MS5837_GetData();
}

const IMU_Data_t* IMU_data_get(void) {
    return IMU_GetData();
}
