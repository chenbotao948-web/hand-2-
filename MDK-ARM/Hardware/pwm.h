#ifndef __PWM_H
#define __PWM_H

#include "main.h"

void PWM_Start(void);
void PWM_LeftSpeed(uint32_t speed);
void PWM_RightSpeed(uint32_t speed);
void PWM_Init(void);
void Car_Stop(void);
#endif
