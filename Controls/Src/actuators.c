#include "actuators.h"
#include "sensor_process.h"
#include "valve_ctrl.h"
#include <stdbool.h>

void Actuators_Init(void){ }

void Actuators_SetMotorPwm(int16_t pwm){
    SetMotorSpeed((uint16_t)pwm);
}

void Actuators_FairingRelease(void){ fairing_release(); }
void Actuators_ValveOpen(void){ valve_open(); }
void Actuators_ValveClose(void){ valve_close(); }
void Actuators_ValveOpenFor(uint32_t ms){ valve_open_for(ms); }
void Actuators_LedToggle(void){
    static bool led_on = false;
    if(led_on){
        LEDstatus_off();
        led_on = false;
    } else {
        LEDstatus_on();
        led_on = true;
    }
}
