#ifndef __PID_H
#define __PID_H

#include "main.h"

/* 20ms PID 参数，可改初值后重新编译，也可在调试器中临时观察和修改。 */
extern volatile float pid_kp;
extern volatile float pid_ki;
extern volatile float pid_kd;
extern volatile float pid_base_speed_us;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float base_speed_us;
    uint8_t running;
} PID_Parameters;

void PID_Init(void);
void PID_Start(void);
void PID_Stop(void);
uint8_t PID_IsRunning(void);
uint8_t PID_SetKp(float value);
uint8_t PID_SetKi(float value);
uint8_t PID_SetKd(float value);
uint8_t PID_SetBaseSpeed(float value);
void PID_GetParameters(PID_Parameters *parameters);
void PID_Update20ms(void);

#endif
