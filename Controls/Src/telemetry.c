#include "telemetry.h"
#include <stdio.h>
#include <stdbool.h>

/*
实现简单遥测系统，用于收集和打印关键状态变量。
*/
static struct { 
    float z;  //滤波后深度
    float v;  //速度
    int mission_state; // 任务状态，参考 mission_manager.h 中的 mission_state_t
    int balloon_state; // 气球状态
    float v_ref; // 参考速度
    int16_t pwm; // PWM值
    float p_bag; // 气囊压力
    float p_water; // 水压
    float dPdt; // 压力变化率
    float duty; // 占空比
    float z_target; // 目标深度
} g_tlm;

void Telemetry_Init(void){ }

void Telemetry_SetDepth(float z, float v, float z_target){
    g_tlm.z=z;
    g_tlm.v=v;
    g_tlm.z_target=z_target;
}

void Telemetry_SetMissionState(int s){
    g_tlm.mission_state=s; 
}
void Telemetry_SetBalloon(int b){
    g_tlm.balloon_state=b; 
}
void Telemetry_SetControl(float v_ref, int16_t pwm){
    g_tlm.v_ref=v_ref;
    g_tlm.pwm=pwm;
}
void Telemetry_SetPressures(float p_bag, float p_water, float dPdt, float duty){
    g_tlm.p_bag=p_bag;
    g_tlm.p_water=p_water;
    g_tlm.dPdt=dPdt;
    g_tlm.duty=duty;
}

void Telemetry_Publish(uint32_t now_ms){
    (void)now_ms; 
    printf("TLM:\r\n z=%.2f z_target=%.2f v=%.3f v_ref = %.3f \r\nmission_state = %d balloon_state = %d \r\npwm = %d p_bag = %.2f p_water = %.2f dPdt = %.2f duty = %.2f\r\n",
        g_tlm.z,g_tlm.z_target, g_tlm.v, g_tlm.v_ref, g_tlm.mission_state, g_tlm.balloon_state, g_tlm.pwm, g_tlm.p_bag, g_tlm.p_water, g_tlm.dPdt, g_tlm.duty);
}
