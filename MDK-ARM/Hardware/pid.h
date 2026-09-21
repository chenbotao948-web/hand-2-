#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "imu.h"

/* 从 Track=1 起计时；串口可修改 RAM 中的实际值。 */
#define PID_ANGLE_WAIT_DEFAULT_MS 20000U
/* 限制在 HAL_GetTick() 无符号差值可靠判断的半个回绕周期内。 */
#define PID_ANGLE_WAIT_MAX_MS 0x7FFFFFFFU

typedef enum
{
    PID_STAGE_STOP = 0,
    PID_STAGE_CENTER,
    PID_STAGE_ANGLE,
    PID_STAGE_EDGE,
    PID_STAGE_MANUAL,
    PID_STAGE_FAULT
} PID_Stage;

typedef enum
{
    PID_ANGLE_LEFT, PID_ANGLE_RIGHT, PID_ANGLE_KP,
    PID_ANGLE_KI, PID_ANGLE_KD, PID_ANGLE_BASE
} PID_AngleParameter;

typedef struct
{
    IMU_State imu;
    PID_Stage stage;
    float initial_deg, delta_deg;
    float left_deg, right_deg, kp, ki, kd, base_us;
    uint32_t elapsed_ms;
    uint32_t wait_ms;
} PID_AngleStatus;

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
uint8_t PID_Start(void);
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
uint8_t PID_SetAngleParameter(PID_AngleParameter parameter, float value);
uint8_t PID_SetAngleWait(uint32_t wait_ms);
void PID_GetAngleStatus(PID_AngleStatus *status);
uint8_t PID_TakeImuFault(void);
PID_Stage PID_GetStage(void);

#endif
