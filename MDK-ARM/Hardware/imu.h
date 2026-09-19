#ifndef IMU_H
#define IMU_H

#include "main.h"

#define IMU_SAMPLE_PERIOD_MS   10U
#define IMU_STALE_MS          100U
/* 统一成车体左转为正。若 Angle? 显示左转为负，停车后改成 -1.0f。 */
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
float IMU_WrapAngle(float degrees);

#endif
