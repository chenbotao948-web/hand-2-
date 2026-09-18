#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef enum
{
    PID_MODE_CENTER = 0,
    PID_MODE_RIGHT_EDGE = 1
} PID_Mode;

/* 原白线中心循迹参数。 */
extern volatile float pid_kp;
extern volatile float pid_ki;
extern volatile float pid_kd;
extern volatile float pid_base_speed_us;

/* 右侧边界循迹参数，与原循迹参数独立。 */
extern volatile float edge_kp;
extern volatile float edge_ki;
extern volatile float edge_kd;
extern volatile float edge_base_speed_us;
extern volatile float edge_lost_extra_us;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float base_speed_us;
    float lost_extra_us;
    PID_Mode mode;
    uint8_t running;
} PID_Parameters;

void PID_Init(void);
void PID_Start(void);
void PID_Stop(void);
uint8_t PID_IsRunning(void);
uint8_t PID_SetMode(PID_Mode mode);
PID_Mode PID_GetMode(void);
uint8_t PID_SetKp(float value);
uint8_t PID_SetKi(float value);
uint8_t PID_SetKd(float value);
uint8_t PID_SetBaseSpeed(float value);
uint8_t PID_SetLostExtra(float value);
void PID_GetParameters(PID_Parameters *parameters);
void PID_Update20ms(void);

#endif
