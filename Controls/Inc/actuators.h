#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" { 
#endif

// 封装bsp层的执行函数，控制层仅调用此处的API

void Actuators_Init(void);
void Actuators_SetMotorPwm(int16_t pwm);
void Actuators_FairingRelease(void);
void Actuators_ValveOpen(void);
void Actuators_ValveClose(void);
void Actuators_ValveOpenFor(uint32_t ms);
void Actuators_LedToggle(void);

#ifdef __cplusplus
}
#endif
