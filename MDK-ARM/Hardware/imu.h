#ifndef IMU_H
#define IMU_H

#include "main.h"

#define IMU_SAMPLE_PERIOD_MS   10U
#define IMU_STALE_MS          100U
/* 当前安装下原始 Yaw 左转减小；TurnTo/AngleTarget 的正号由控制端换算。 */
#define IMU_YAW_SIGN            1.0f

typedef struct
{
    float yaw_deg;
    uint32_t sample_ms;
    uint32_t fault_count;
    uint32_t max_gap_ms;  /* 校准后最大采样间隔，便于同时刷新屏幕/串口时检查 */
    uint8_t ready;
    uint8_t healthy;
} IMU_State;

void IMU_Init(void);
void IMU_Task(void);
void IMU_GetState(IMU_State *state);
/* 返回 1 表示角度有效；返回 0 时不修改 *angle。 */
uint8_t IMU_GetAngle(float *angle);
float IMU_WrapAngle(float degrees);

#endif
