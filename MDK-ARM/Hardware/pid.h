#ifndef __PID_H
#define __PID_H

#include "main.h"

/* 20ms PID 参数，可改初值后重新编译，也可在调试器中临时观察和修改。 */
extern volatile float pid_kp;
extern volatile float pid_ki;
extern volatile float pid_kd;
extern volatile float pid_base_speed_us;

void PID_Init(void);
void PID_Start(void);
void PID_Stop(void);
uint8_t PID_IsRunning(void);
void PID_Update20ms(void);

#endif
