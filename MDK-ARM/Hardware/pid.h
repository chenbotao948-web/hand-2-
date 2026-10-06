#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "imu.h"


/* 中心循迹 */
void PID_UpdateCenter(uint8_t white_bits);

/* 角度控制 */
void PID_UpdateAngle(float yaw);

/* 原地转向相关 */
void PID_ResetTurnHistory(void);

/* Angle 阶段是否已经完成原地转向 */
extern uint8_t angle_turn_done;

//目标角度
extern float angle_target_yaw;   // 目标 yaw

void PID_ResetEdgeHistory(void);//边沿参数清除|

void PID_UpdateRightEdge(uint8_t white_bits);//边沿检测代码
/*中心pid*/
uint8_t PID_SetKp(float value);
uint8_t PID_SetKi(float value);
uint8_t PID_SetKd(float value);
uint8_t PID_SetCenterBaseSpeed(float value);
/*读取*/
float PID_GetKp(void);


/*边沿pid*/
uint8_t PID_SetEdgeBaseSpeed(float value);
uint8_t PID_SetEKp(float value);
uint8_t PID_SetEKi(float value);
uint8_t PID_SetEKd(float value);

typedef struct
{
    float kp;
    float ki;
    float kd;
    float base_speed_us;
} PID_Parameters;//结构体

void PID_GetParameters(PID_Parameters *parameters);
#endif