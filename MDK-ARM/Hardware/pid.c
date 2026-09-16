#include "pid.h"
#include "ir.h"
#include "pwm.h"

/*
 * 第一版循迹参数，适用于固定 20ms 控制周期。
 * 调参顺序：先只调 Kp，再增加 Kd，最后确有需要时再增加 Ki。
 */
volatile float pid_kp = 20.0f;
volatile float pid_ki = 0.0f;
volatile float pid_kd = 0.0f;
volatile float pid_base_forward_us = 100.0f;

#define PID_NEUTRAL_US          1500.0f
#define PID_FORWARD_MAX_US       250.0f
#define PID_I_ACTIVE_ERROR         1.0f
#define PID_I_OUTPUT_LIMIT_US     20.0f

static volatile uint8_t pid_running = 0;
static float pid_error_sum = 0.0f;
static float pid_last_error = 0.0f;
static uint8_t pid_has_last_error = 0;
static uint8_t pid_has_seen_line = 0;

static float PID_LimitFloat(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

static void PID_ResetHistory(void)
{
    pid_error_sum = 0.0f;
    pid_last_error = 0.0f;
    pid_has_last_error = 0;
}

static void PID_SetStopPulse(void)
{
    PWM_LeftSpeed(1500U);
    PWM_RightSpeed(1500U);
}

static void PID_ApplyForward(float left_forward, float right_forward)
{
    uint32_t left_pulse;
    uint32_t right_pulse;

    left_forward = PID_LimitFloat(left_forward, 0.0f, PID_FORWARD_MAX_US);
    right_forward = PID_LimitFloat(right_forward, 0.0f, PID_FORWARD_MAX_US);

    /* 左侧减小脉宽前进，右侧增大脉宽前进。 */
    left_pulse = (uint32_t)(PID_NEUTRAL_US - left_forward + 0.5f);
    right_pulse = (uint32_t)(PID_NEUTRAL_US + right_forward + 0.5f);
    PWM_LeftSpeed(left_pulse);
    PWM_RightSpeed(right_pulse);
}

static float PID_CalculatePosition(uint8_t white_bits)
{
    uint8_t i;
    uint8_t count = 0;
    uint16_t position_sum = 0;

    /* bit7 是最左，bit0 是最右；从左到右依次赋予 1~8 的权重。 */
    for (i = 0; i < 8; i++)
    {
        if ((white_bits & (uint8_t)(0x80U >> i)) != 0U)
        {
            position_sum = (uint16_t)(position_sum + i + 1U);
            count++;
        }
    }

    return (float)position_sum / (float)count;
}

void PID_Init(void)
{
    pid_running = 0;
    PID_ResetHistory();
    pid_has_seen_line = 0;
    PID_SetStopPulse();
}

void PID_Start(void)
{
    /* 最后才开放中断更新，避免复位状态时被 TIM3 使用到一半。 */
    pid_running = 0;
    PID_ResetHistory();
    pid_has_seen_line = 0;
    PID_SetStopPulse();
    pid_running = 1;
}

void PID_Stop(void)
{
    /* 先禁止中断更新，再清历史和停车。 */
    pid_running = 0;
    PID_ResetHistory();
    pid_has_seen_line = 0;
    PID_SetStopPulse();
}

uint8_t PID_IsRunning(void)
{
    return pid_running;
}

void PID_Update20ms(void)
{
    uint8_t raw_state;
    uint8_t white_bits;
    float position;
    float error;
    float derivative = 0.0f;
    float correction;
    float correction_limit;
    float base_forward;
    float kp;
    float ki;
    float kd;

    if (pid_running == 0U)
    {
        return;
    }

    raw_state = IR_Read();
    white_bits = (uint8_t)(~raw_state);  /* 原始 0 表示白线，转换后 1 表示白线 */
    kp = pid_kp;
    ki = pid_ki;
    kd = pid_kd;
    base_forward = PID_LimitFloat(pid_base_forward_us,
                                  0.0f,
                                  PID_FORWARD_MAX_US);

    if (white_bits == 0x00U)
    {
        /* 全部位于黑底：清历史。曾经找到过线时保持上一拍输出。 */
        PID_ResetHistory();
        if (pid_has_seen_line == 0U)
        {
            PID_SetStopPulse();
        }
        return;
    }

    if (white_bits == 0xFFU)
    {
        /* 全部位于白色区域：按居中直行处理，并清除 PID 历史。 */
        PID_ResetHistory();
        pid_has_seen_line = 1;
        PID_ApplyForward(base_forward, base_forward);
        return;
    }

    position = PID_CalculatePosition(white_bits);
    error = 4.5f - position;

    if ((ki > 0.0f) &&
        (error >= -PID_I_ACTIVE_ERROR) &&
        (error <= PID_I_ACTIVE_ERROR))
    {
        float error_sum_limit = PID_I_OUTPUT_LIMIT_US / ki;
        pid_error_sum += error;
        pid_error_sum = PID_LimitFloat(pid_error_sum,
                                       -error_sum_limit,
                                       error_sum_limit);
    }
    else
    {
        pid_error_sum = 0.0f;
    }

    /* 中间两路同时见到白线时，主动消除残留积分。 */
    if ((white_bits & 0x18U) == 0x18U)
    {
        pid_error_sum = 0.0f;
    }

    if (pid_has_last_error != 0U)
    {
        derivative = error - pid_last_error;
    }
    pid_last_error = error;
    pid_has_last_error = 1;

    correction = kp * error +
                 ki * pid_error_sum +
                 kd * derivative;

    correction_limit = base_forward;
    if ((PID_FORWARD_MAX_US - base_forward) < correction_limit)
    {
        correction_limit = PID_FORWARD_MAX_US - base_forward;
    }
    correction = PID_LimitFloat(correction,
                                -correction_limit,
                                correction_limit);

    /* correction > 0 时左侧减速、右侧加速，使车向左纠偏。 */
    PID_ApplyForward(base_forward - correction,
                     base_forward + correction);
    pid_has_seen_line = 1;
}
