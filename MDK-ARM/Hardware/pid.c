#include "pid.h"
#include "ir.h"
#include "pwm.h"

/* 第一套：白线中心循迹参数，控制周期固定为 20ms。 */
volatile float pid_kp = 40.0f;
volatile float pid_ki = 0.0f;
volatile float pid_kd = 0.0f;
volatile float pid_base_speed_us = 80.0f;

/* 第二套：从右向左搜索白线右侧边界，参数独立调节。 */
volatile float edge_kp = 30.0f;
volatile float edge_ki = 0.0f;
volatile float edge_kd = 0.0f;
volatile float edge_base_speed_us = 55.0f;
volatile float edge_lost_extra_us = 30.0f;

/* 左右容差单位 deg；参数仅存 RAM。角度 PID 按每拍累加/差分，周期 20ms。角度pid */
static float angle_left_deg = 10.0f, angle_right_deg = 10.0f;
static float angle_kp = 2.0f, angle_ki = 0.0f, angle_kd = 0.0f;
static float angle_base_us = 80.0f;
static volatile uint32_t angle_wait_ms = PID_ANGLE_WAIT_DEFAULT_MS;
static float angle_sum, angle_last_error;
static uint8_t angle_has_last_error;
static volatile PID_Stage pid_stage = PID_STAGE_STOP;
static volatile uint8_t imu_fault_pending;
static float initial_yaw;
static uint32_t start_ms, start_fault_count;
static uint8_t has_reference;

#define PID_STOP_US                 1500.0f
#define PID_MAX_US                   250.0f
#define PID_REVERSE_DEADZONE_US       50.0f
#define PID_GAIN_MAX                1000.0f
#define PID_I_ACTIVE_ERROR             1.0f
#define PID_I_OUTPUT_LIMIT_US          20.0f
#define EDGE_TARGET_POSITION            5.5f//目标位置

static volatile uint8_t pid_running = 0U;
static volatile PID_Mode pid_mode = PID_MODE_CENTER;

static float center_error_sum = 0.0f;
static float center_last_error = 0.0f;
static uint8_t center_has_last_error = 0U;

static float edge_error_sum = 0.0f;
static float edge_last_error = 0.0f;
static uint8_t edge_has_last_error = 0U;
static float edge_last_position = EDGE_TARGET_POSITION;
static uint8_t edge_has_last_position = 0U;

static float PID_Limit(float value, float low, float high)//把值限制在[low, high]区间内
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

static void PID_ResetCenterHistory(void)
{
    center_error_sum = 0.0f;
    center_last_error = 0.0f;
    center_has_last_error = 0U;
}

static void PID_ResetEdgePidHistory(void)
{
    edge_error_sum = 0.0f;
    edge_last_error = 0.0f;
    edge_has_last_error = 0U;
}

static void PID_ResetEdgeHistory(void)//这里清除上一个位置了，实际效果不知道好不好
{
    PID_ResetEdgePidHistory();
    edge_last_position = EDGE_TARGET_POSITION;
    edge_has_last_position = 0U;
}

static void PID_ResetAllHistory(void)
{
    PID_ResetCenterHistory();
    PID_ResetEdgeHistory();
    angle_sum = angle_last_error = 0.0f;
    angle_has_last_error = 0U;
}

static uint8_t PID_ValueInRange(float value, float high)//判断值是否合法
{
    /* value != value 可以识别 NaN；正负无穷会被范围判断拒绝。 */
    if ((value != value) || (value < 0.0f) || (value > high))
    {
        return 0U;
    }
    return 1U;
}

static uint32_t PID_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void PID_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Car_Stop(void)
{
    PWM_LeftSpeed(1500U);
    PWM_RightSpeed(1500U);
}

static void PID_ApplyForward(float left_forward, float right_forward)
{
    uint32_t left_pulse;
    uint32_t right_pulse;

    /* 负速度表示反转，先补偿实测死区，再限制最终输出幅度。 */
    if (left_forward < 0.0f)
    {
        left_forward -= PID_REVERSE_DEADZONE_US;
    }
    if (right_forward < 0.0f)
    {
        right_forward -= PID_REVERSE_DEADZONE_US;
    }

    left_forward = PID_Limit(left_forward, -PID_MAX_US, PID_MAX_US);
    right_forward = PID_Limit(right_forward, -PID_MAX_US, PID_MAX_US);

    left_pulse = (uint32_t)(PID_STOP_US - left_forward + 0.5f);
    right_pulse = (uint32_t)(PID_STOP_US + right_forward + 0.5f);
    PWM_LeftSpeed(left_pulse);
    PWM_RightSpeed(right_pulse);
}

static float PID_CalculateCenterPosition(uint8_t white_bits)
{
    uint8_t i;
    uint8_t count = 0U;
    uint16_t position_sum = 0U;

    /* bit7 是最左，bit0 是最右；从左到右依次赋予 1~8 的权重。 */
    for (i = 0U; i < 8U; i++)
    {
        if ((white_bits & (uint8_t)(0x80U >> i)) != 0U)
        {
            position_sum = (uint16_t)(position_sum + i + 1U);
            count++;
        }
    }

    return (float)position_sum / (float)count;
}

static uint8_t PID_FindRightEdge(uint8_t white_bits, float *position)//边沿检测
{
    uint8_t left_sensor;
    uint8_t left_mask;
    uint8_t right_mask;

    /* 从第 8 路向第 1 路搜索，取最右侧的“左白、右黑”交界。 */
    for (left_sensor = 7U; left_sensor > 0U; left_sensor--)//从7开始每次减1，减少到1
    {
        left_mask = (uint8_t)(0x80U >> (left_sensor - 1U));
        right_mask = (uint8_t)(0x80U >> left_sensor);
        if (((white_bits & left_mask) != 0U) &&
            ((white_bits & right_mask) == 0U))
        {
            *position = (float)left_sensor + 0.5f;
            return 1U;
        }
    }

    return 0U;
}

static void PID_UpdateCenter(uint8_t white_bits)
{
    float position;
    float error;
    float derivative = 0.0f;
    float correction;
    float kp = pid_kp;
    float ki = pid_ki;
    float kd = pid_kd;
    float base_speed = pid_base_speed_us;

    if (white_bits == 0x00U)
    {
        /* 全部位于黑底：清除原算法历史，保持上一拍电机输出。 */
        PID_ResetCenterHistory();
        return;
    }

    position = PID_CalculateCenterPosition(white_bits);
    error = 4.5f - position;

    if ((ki > 0.0f) &&
        (error >= -PID_I_ACTIVE_ERROR) &&
        (error <= PID_I_ACTIVE_ERROR))
    {
        float error_sum_limit = PID_I_OUTPUT_LIMIT_US / ki;
        center_error_sum += error;
        center_error_sum = PID_Limit(center_error_sum,
                                     -error_sum_limit,
                                     error_sum_limit);
    }
    else
    {
        center_error_sum = 0.0f;
    }

    /* 原算法保持中间两路同时见到白线时清除积分的行为。 */
    if ((white_bits & 0x18U) == 0x18U)
    {
        center_error_sum = 0.0f;
    }

    if (center_has_last_error != 0U)
    {
        derivative = error - center_last_error;
    }
    center_last_error = error;
    center_has_last_error = 1U;

    correction = kp * error +
                 ki * center_error_sum +
                 kd * derivative;

    PID_ApplyForward(base_speed - correction,
                     base_speed + correction);
}

static void PID_UpdateRightEdge(uint8_t white_bits)
{
    float position;
    float error;
    float derivative = 0.0f;
    float correction;
    float kp = edge_kp;
    float ki = edge_ki;
    float kd = edge_kd;
    float base_speed = edge_base_speed_us;//用全局变量赋值，没有问题

    /* 第 4、5 路为白色且第 6 路为黑色时，边界正好在目标位置。 */
    if (((white_bits & 0x18U) == 0x18U) &&
        ((white_bits & 0x04U) == 0U))
    {
        PID_ResetEdgePidHistory();
        edge_last_position = EDGE_TARGET_POSITION;//这里是宏定义5.5
        edge_has_last_position = 1U;//标记这个位置是有效的，后面万一丢线，可以用它来判断方向。虽然感觉没用，毕竟情况特殊
        PID_ApplyForward(base_speed, base_speed);
        return;
    }

    if (PID_FindRightEdge(white_bits, &position) == 0U)
    {
        /* 丢线时保留有效边界位置，固定加大最后一次的转向量。 */
        PID_ResetEdgePidHistory();
        if (edge_has_last_position == 0U)
        {
            return;
        }//标记生效

        error = EDGE_TARGET_POSITION - edge_last_position;
        if (error > 0.0f)//中心目标是5.5
        {
            correction = kp * error + edge_lost_extra_us;
        }
        else if (error < 0.0f)
        {
            correction = kp * error - edge_lost_extra_us;
        }
        else
        {
            /* 按实际赛道不会发生；没有方向时保持上一拍输出。 */
            return;
        }

        correction = PID_Limit(correction, -PID_MAX_US, PID_MAX_US);
        PID_ApplyForward(base_speed - correction,
                         base_speed + correction);
        return;
    }

    edge_last_position = position;
    edge_has_last_position = 1U;
    error = EDGE_TARGET_POSITION - position;

    if ((ki > 0.0f) &&
        (error >= -PID_I_ACTIVE_ERROR) &&
        (error <= PID_I_ACTIVE_ERROR))
    {
        float error_sum_limit = PID_I_OUTPUT_LIMIT_US / ki;
        edge_error_sum += error;
        edge_error_sum = PID_Limit(edge_error_sum,
                                   -error_sum_limit,
                                   error_sum_limit);
    }
    else
    {
        edge_error_sum = 0.0f;
    }//超出范围不要

    if (edge_has_last_error != 0U)
    {
        derivative = error - edge_last_error;
    }
    edge_last_error = error;
    edge_has_last_error = 1U;

    correction = kp * error +
                 ki * edge_error_sum +
                 kd * derivative;

    PID_ApplyForward(base_speed - correction,
                     base_speed + correction);
}

void PID_Init(void)
{
    pid_running = 0U;
    pid_mode = PID_MODE_CENTER;
    pid_stage = PID_STAGE_STOP;
    angle_wait_ms = PID_ANGLE_WAIT_DEFAULT_MS;
    has_reference = imu_fault_pending = 0U;
    PID_ResetAllHistory();
    Car_Stop();
}

uint8_t PID_Start(void)
{
    IMU_State imu;
    uint32_t key = PID_EnterCritical();
    uint32_t now = HAL_GetTick();
    IMU_GetState(&imu);
    if (!imu.ready || !imu.healthy ||
        (uint32_t)(now - imu.sample_ms) > IMU_STALE_MS)
    {
        PID_Stop();
        PID_ExitCritical(key);
        return 0U;
    }
    pid_running = 0U;
    PID_ResetAllHistory();
    Car_Stop();
    initial_yaw = imu.yaw_deg;
    start_ms = now;
    start_fault_count = imu.fault_count;
    has_reference = 1U;
    pid_mode = PID_MODE_CENTER;
    pid_stage = PID_STAGE_CENTER;
    pid_running = 1U;
    PID_ExitCritical(key);
    return 1U;
}

void PID_Stop(void)
{
    uint32_t key = PID_EnterCritical();
    pid_running = 0U;
    pid_stage = PID_STAGE_STOP;
    has_reference = 0U;
    PID_ResetAllHistory();
    Car_Stop();
    PID_ExitCritical(key);
}

uint8_t PID_IsRunning(void)
{
    return pid_running;
}

uint8_t PID_SetMode(PID_Mode mode)
{
    uint32_t primask;

    if ((mode != PID_MODE_CENTER) && (mode != PID_MODE_RIGHT_EDGE))
    {
        return 0U;
    }

    primask = PID_EnterCritical();
    /* 即使 Mode 值未变，也要取消自动流程。停车时只选择，不起跑。 */
    pid_mode = mode;
    pid_stage = pid_running ? PID_STAGE_MANUAL : PID_STAGE_STOP;
    has_reference = 0U;
    PID_ResetAllHistory();
    PID_ExitCritical(primask);
    return 1U;
}

PID_Mode PID_GetMode(void)
{
    return pid_mode;
}

uint8_t PID_SetKp(float value)
{
    uint32_t primask;

    if (PID_ValueInRange(value, PID_GAIN_MAX) == 0U)
    {
        return 0U;
    }
    primask = PID_EnterCritical();
    if (pid_mode == PID_MODE_RIGHT_EDGE)
    {
        edge_kp = value;
        PID_ResetEdgePidHistory();
    }
    else
    {
        pid_kp = value;
        PID_ResetCenterHistory();
    }
    PID_ExitCritical(primask);
    return 1U;
}

uint8_t PID_SetKi(float value)
{
    uint32_t primask;

    if (PID_ValueInRange(value, PID_GAIN_MAX) == 0U)
    {
        return 0U;
    }
    primask = PID_EnterCritical();
    if (pid_mode == PID_MODE_RIGHT_EDGE)
    {
        edge_ki = value;
        PID_ResetEdgePidHistory();
    }
    else
    {
        pid_ki = value;
        PID_ResetCenterHistory();
    }
    PID_ExitCritical(primask);
    return 1U;
}

uint8_t PID_SetKd(float value)
{
    uint32_t primask;

    if (PID_ValueInRange(value, PID_GAIN_MAX) == 0U)
    {
        return 0U;
    }
    primask = PID_EnterCritical();
    if (pid_mode == PID_MODE_RIGHT_EDGE)
    {
        edge_kd = value;
        PID_ResetEdgePidHistory();
    }
    else
    {
        pid_kd = value;
        PID_ResetCenterHistory();
    }
    PID_ExitCritical(primask);
    return 1U;
}

uint8_t PID_SetBaseSpeed(float value)
{
    uint32_t primask;

    if (PID_ValueInRange(value, PID_MAX_US) == 0U)
    {
        return 0U;
    }
    primask = PID_EnterCritical();
    if (pid_mode == PID_MODE_RIGHT_EDGE)
    {
        edge_base_speed_us = value;
    }
    else
    {
        pid_base_speed_us = value;
    }
    PID_ExitCritical(primask);
    return 1U;
}

uint8_t PID_SetLostExtra(float value)
{
    uint32_t primask;

    if (PID_ValueInRange(value, PID_MAX_US) == 0U)
    {
        return 0U;
    }
    primask = PID_EnterCritical();
    edge_lost_extra_us = value;
    PID_ExitCritical(primask);
    return 1U;
}

void PID_GetParameters(PID_Parameters *parameters)
{
    uint32_t primask;

    if (parameters == NULL)//防止空指针
    {
        return;
    }

    primask = PID_EnterCritical();
    parameters->mode = pid_mode;
    if (pid_mode == PID_MODE_RIGHT_EDGE)
    {
        parameters->kp = edge_kp;
        parameters->ki = edge_ki;
        parameters->kd = edge_kd;
        parameters->base_speed_us = edge_base_speed_us;
    }
    else
    {
        parameters->kp = pid_kp;
        parameters->ki = pid_ki;
        parameters->kd = pid_kd;
        parameters->base_speed_us = pid_base_speed_us;
    }
    parameters->lost_extra_us = edge_lost_extra_us;
    parameters->running = pid_running;
    PID_ExitCritical(primask);
}

uint8_t PID_SetAngleParameter(PID_AngleParameter parameter, float value)
{
    uint32_t key;
    float limit = (parameter == PID_ANGLE_BASE) ? PID_MAX_US : PID_GAIN_MAX;
    if ((unsigned)parameter > (unsigned)PID_ANGLE_BASE) return 0U;
    if (!PID_ValueInRange(value, limit)) return 0U;
    if ((parameter == PID_ANGLE_LEFT || parameter == PID_ANGLE_RIGHT) && value >= 180.0f)
        return 0U;
    key = PID_EnterCritical();
    switch (parameter)
    {
        case PID_ANGLE_LEFT: angle_left_deg = value; break;
        case PID_ANGLE_RIGHT: angle_right_deg = value; break;
        case PID_ANGLE_KP: angle_kp = value; break;
        case PID_ANGLE_KI: angle_ki = value; break;
        case PID_ANGLE_KD: angle_kd = value; break;
        case PID_ANGLE_BASE: angle_base_us = value; break;
        default: break;
    }
    if (parameter >= PID_ANGLE_KP && parameter <= PID_ANGLE_KD)
    {
        angle_sum = angle_last_error = 0.0f;
        angle_has_last_error = 0U;
    }
    PID_ExitCritical(key);
    return 1U;
}

uint8_t PID_SetAngleWait(uint32_t wait_ms)
{
    uint32_t key;
    if (wait_ms > PID_ANGLE_WAIT_MAX_MS) return 0U;
    key = PID_EnterCritical();
    angle_wait_ms = wait_ms;
    PID_ExitCritical(key);
    return 1U;
}

void PID_GetAngleStatus(PID_AngleStatus *status)
{
    uint32_t key = PID_EnterCritical();
    IMU_GetState(&status->imu);
    status->stage = pid_stage;
    status->initial_deg = has_reference ? initial_yaw : 0.0f;
    status->delta_deg = has_reference ? IMU_WrapAngle(status->imu.yaw_deg - initial_yaw) : 0.0f;
    status->elapsed_ms = has_reference ? (uint32_t)(HAL_GetTick() - start_ms) : 0U;
    status->left_deg = angle_left_deg;
    status->right_deg = angle_right_deg;
    status->kp = angle_kp; status->ki = angle_ki; status->kd = angle_kd;
    status->base_us = angle_base_us;
    status->wait_ms = angle_wait_ms;
    PID_ExitCritical(key);
}

PID_Stage PID_GetStage(void)
{
    return pid_stage;
}

uint8_t PID_TakeImuFault(void)
{
    uint32_t key = PID_EnterCritical();
    uint8_t pending = imu_fault_pending;
    imu_fault_pending = 0U;
    PID_ExitCritical(key);
    return pending;
}

static void PID_UpdateAngle(float delta)
{
    /* delta 左偏为正，此时应右转：左轮加速、右轮减速。 */
    float error = -delta;
    float derivative = 0.0f, correction;
    if (angle_ki > 0.0f)
    {
        angle_sum = PID_Limit(angle_sum + error,
                             -PID_I_OUTPUT_LIMIT_US / angle_ki,
                              PID_I_OUTPUT_LIMIT_US / angle_ki);
    }
    else angle_sum = 0.0f;
    if (angle_has_last_error)
        derivative = IMU_WrapAngle(error - angle_last_error);
    angle_last_error = error;
    angle_has_last_error = 1U;
    correction = angle_kp * error + angle_ki * angle_sum + angle_kd * derivative;
    PID_ApplyForward(angle_base_us - correction, angle_base_us + correction);
}

void PID_Update20ms(void)
{
    uint8_t white_bits;
    IMU_State imu;
    uint32_t now;
    float delta;//算偏航角

    if (!pid_running) return;//停车=1就返回
    white_bits = (uint8_t)(~IR_Read());
    if (pid_stage == PID_STAGE_CENTER || pid_stage == PID_STAGE_ANGLE)
    {
        now = HAL_GetTick();
        IMU_GetState(&imu);
        if (!imu.ready || !imu.healthy || imu.fault_count != start_fault_count ||
            (uint32_t)(now - imu.sample_ms) > IMU_STALE_MS)
        {
            /* 中断仅停车并记录，串口故障报告交给主循环。保留参考角便于查询。 */
            pid_running = 0U;
            pid_stage = PID_STAGE_FAULT;
            imu_fault_pending = 1U;
            PID_ResetAllHistory();
            Car_Stop();
            return;
        }
        delta = IMU_WrapAngle(imu.yaw_deg - initial_yaw);
        if (pid_stage == PID_STAGE_CENTER &&
            (uint32_t)(now - start_ms) >= angle_wait_ms && white_bits == 0U &&
            delta >= -angle_right_deg && delta <= angle_left_deg)/* 等待时间到  8 路全黑 丢线在容差内*/
        {
            PID_ResetAllHistory();
            pid_stage = PID_STAGE_ANGLE;//角度pid模式
        }
        if (pid_stage == PID_STAGE_ANGLE)//进入角度pid，如果没有遇到线
        {
            if (white_bits == 0U)
            {
                PID_UpdateAngle(delta);
                return;
            }
            /* 任意一路白色即接管；没有有效边界时沿用 PID2 保持上一拍的行为。 */
            PID_ResetAllHistory();
            pid_mode = PID_MODE_RIGHT_EDGE;//遇到切换模式
            pid_stage = PID_STAGE_EDGE;
        }
    }
    if (pid_mode == PID_MODE_RIGHT_EDGE) PID_UpdateRightEdge(white_bits);
    else PID_UpdateCenter(white_bits);//结尾判断切换模式
}
