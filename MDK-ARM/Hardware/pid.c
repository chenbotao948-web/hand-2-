#include "pid.h"
#include "ir.h"
#include "pwm.h"
#include "stage.h"
#include "usart.h"
/* 第一套：白线中心循迹参数，控制周期固定为 20ms。 */
volatile float pid_kp = 40.0f;
volatile float pid_ki = 0.0f;
volatile float pid_kd = 0.0f;
volatile float pid_base_speed_us = 55.0f;

static float center_error_sum = 0.0f;
static float center_last_error = 0.0f;
static uint8_t center_has_last_flag = 0U;


 /* 角度 PID 参数 */
 uint8_t angle_turn_done = 0U;
static float angle_left_deg  = 10.0f;   // 左容差
static float angle_right_deg = 10.0f;   // 右容差
static float angle_kp = 2.0f;
static float angle_ki = 0.0f;//直走参数
static float angle_kd = 0.0f;
static float angle_base_us = 80.0f;     // 角度直行时的基础速度

static float angle_sum = 0.0f;          // 积分累加
static float angle_last_error = 0.0f;   // 上一拍误差
static uint8_t angle_has_last_error = 0U;

float angle_target_yaw = 0.0f;   // 目标 yaw
static uint8_t angle_has_target = 0U;   // 是否已经记录目标

static uint8_t use_angle_pid = 0U;      // 0=循迹，1=角度

#define PID_I_OUTPUT_LIMIT_US         50.0f
#define PID_I_ACTIVE_ERROR             1.0f
#define PID_STOP_US                 1500.0f//不用改
#define PID_MAX_US                   250.0f
#define PID_REVERSE_DEADZONE_US       50.0f
#define angle_tol_deg      			  10.0f

/* 原地转向 PID */
static float turn_kp = 3.0f;
static float turn_ki = 0.0f;
static float turn_kd = 0.0f;//原地转弯参数

static float turn_max_us = 80.0f;     // 最大转向输出
static float turn_tol_deg = 10.0f;     // 允许误差，例如 ±5°

static float turn_sum = 0.0f;
static float turn_last_error = 0.0f;

static uint8_t turn_has_last_error = 0U;
static uint8_t turn_stable_ticks = 0U;

/* Angle阶段有没有完成第一次原地转向 */

static volatile uint8_t pid_running = 0U;

/* 原地转向死区补偿 */
static float turn_fwd_comp_us = 40.0f;
static float turn_rev_comp_us = 40.0f;

static uint8_t turn_kick_phase = 0U;

#define PID_TURN_KICK_PERIOD_TICKS  5U
#define PID_TURN_KICK_ON_TICKS      2U

static void PID_UpdateTurn(float yaw);
static void PID_ApplyTurn(float output, uint8_t kick);//一些变量提前声明

/* 右边沿 PID 参数 */
static float edge_kp = 30.0f;
static float edge_ki = 0.0f;
static float edge_kd = 0.0f;
static float edge_base_speed_us = 55.0f;
static float edge_lost_extra_us = 30.0f;

static float edge_error_sum = 0.0f;
static float edge_last_error = 0.0f;
static uint8_t edge_has_last_error = 0U;

static float edge_last_position = 5.5f;
static uint8_t edge_has_last_position = 0U;

#define EDGE_TARGET_POSITION 5.5f//标准位置

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
    center_has_last_flag = 0U;
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

static void PID_ApplyForward(float left_forward, float right_forward)//应用
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


 void PID_UpdateCenter(uint8_t white_bits)//计算pid
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

    if ((ki > 0.0f) &&(error >= -PID_I_ACTIVE_ERROR) &&(error <= PID_I_ACTIVE_ERROR))//积分区域
    {
        float error_sum_limit = PID_I_OUTPUT_LIMIT_US / ki;//之后要成KI，所以这里先除了。
        center_error_sum += error;
        center_error_sum = PID_Limit(center_error_sum,-error_sum_limit,error_sum_limit);
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

    if (center_has_last_flag != 0U)//除去第一次的误差变化量
    {
        derivative = error - center_last_error;
    }
    center_last_error = error;
    center_has_last_flag = 1U;

    correction = kp * error +
                 ki * center_error_sum +
                 kd * derivative;

    PID_ApplyForward(base_speed - correction,
                     base_speed + correction);
}


void PID_UpdateAngle(float yaw)//角度pid
{
    float error, derivative = 0.0f, correction;
	if(angle_turn_done == 0U)//没到目标角度先转弯
    {
		PID_UpdateTurn(yaw);
		return;
    }
    /* 目标减当前，再 wrap，保证落在 [-180, 180) */
    error = IMU_WrapAngle(angle_target_yaw - yaw);

    /* 容差内不修，直接直走 */
    if (error >= -angle_tol_deg && error <= angle_tol_deg)
    {
        angle_sum = 0.0f;
        angle_last_error = 0.0f;
        angle_has_last_error = 0U;
        PID_ApplyForward(angle_base_us, angle_base_us);
        return;
    }

    /* 积分 */
    if (angle_ki > 0.0f)
    {
        float limit = 20.0f / angle_ki;
        angle_sum += error;
        angle_sum = PID_Limit(angle_sum, -limit, limit);
    }
    else
    {
        angle_sum = 0.0f;
    }

    /* 微分 */
    if (angle_has_last_error != 0U)
    {
        derivative = IMU_WrapAngle(error - angle_last_error);
    }
    angle_last_error = error;
    angle_has_last_error = 1U;

    correction = angle_kp * error +
                 angle_ki * angle_sum +
                 angle_kd * derivative;

    /* 修正量不能超过基础速度，否则会倒转 */
    correction = PID_Limit(correction, -angle_base_us, angle_base_us);

    PID_ApplyForward(angle_base_us - correction,
                     angle_base_us + correction);
}

void PID_ResetTurnHistory(void)
{
    turn_sum = 0.0f;
    turn_last_error = 0.0f;

    turn_has_last_error = 0U;
    turn_stable_ticks = 0U;//判断是不是稳定在三个节拍
	
	turn_kick_phase = 0U;//测试用来判断是不是还要增加转向。
}

static void PID_ApplyTurn(float output,uint8_t kick)//因为参数不一样，所以要用另外一套
{
	float left_output;
    float right_output;

    /* 先限制PID本身最大输出 */
    output = PID_Limit(output,-turn_max_us,turn_max_us);

    /*
     * 默认不补偿：
     * 左右两边都直接使用 PID output
     */
    left_output = output;
    right_output = output;


    /*
     * output > 0
     *
     * 当前转向方向：
     * 左轮正转
     * 右轮反转
     */
    if ((kick != 0U) && (output > 0.0f))
    {
        left_output =PID_Limit(output + turn_fwd_comp_us,0.0f,turn_max_us);

        right_output =PID_Limit(output + turn_rev_comp_us,0.0f,turn_max_us);
    }


    /*
     * output < 0
     *
     * 当前转向方向：
     * 左轮反转
     * 右轮正转
     */
    else if ((kick != 0U) && (output < 0.0f))
    {
        left_output =PID_Limit(output - turn_rev_comp_us,-turn_max_us,0.0f);

        right_output = PID_Limit(output - turn_fwd_comp_us,-turn_max_us,0.0f);
    }


    PWM_LeftSpeed((uint32_t)(PID_STOP_US - left_output + 0.5f));

    PWM_RightSpeed((uint32_t)(PID_STOP_US - right_output + 0.5f));
}

static void PID_UpdateTurn(float yaw)
{
    float error;
    float magnitude;
    float derivative = 0.0f;

    float output;
    float output_abs;

    uint8_t kick = 0U;

    /* 目标角度 - 当前角度 */
    error = IMU_WrapAngle(angle_target_yaw - yaw);

    /* 求绝对值 */
    magnitude = (error < 0.0f) ? -error : error;


    /* ---------- 已经进入目标角度附近 ---------- */

    if (magnitude <= turn_tol_deg)
    {
        Car_Stop();

        /*
         * 连续3拍都在容差内，
         * 才认为真的转好了。
         *
         * 20ms一拍，3拍就是约60ms。
         */
        turn_stable_ticks++;

        if (turn_stable_ticks >= 3U)
        {
            angle_turn_done = 1U;

            PID_ResetTurnHistory();

            /* 为后面的直行PID清历史 */
            angle_sum = 0.0f;
            angle_last_error = 0.0f;
            angle_has_last_error = 0U;
        }

        return;
    }

    /* 一旦跑出容差，重新计稳定次数 */
    turn_stable_ticks = 0U;


    /* ---------- I ---------- */

    if ((turn_ki > 0.0f) &&
        (magnitude <= 15.0f))
    {
        float limit;

        limit = PID_I_OUTPUT_LIMIT_US / turn_ki;

        turn_sum += error;

        turn_sum = PID_Limit(turn_sum, -limit,limit);
    }
    else
    {
        turn_sum = 0.0f;
    }


    /* ---------- D ---------- */

    if (turn_has_last_error != 0U)
    {
        derivative =IMU_WrapAngle(error - turn_last_error);
    }

    turn_last_error = error;
    turn_has_last_error = 1U;


    /* ---------- PID ---------- */

    output =turn_kp * error +turn_ki * turn_sum + turn_kd * derivative;


    /* ---------- 原地转向 ---------- */
/* PID输出取绝对值 */
	output_abs =(output < 0.0f) ? -output : output;


/*
 * 判断现在是否需要死区补偿
 */
	if ((output_abs > 0.0f) &&

    /* PID输出方向和误差方向一致 */
    (output * error > 0.0f) &&

    /* PID没有已经顶到最大输出 */
    (output_abs < turn_max_us) &&

    /* 当前输出还小于死区补偿量 */
    ((output_abs < turn_fwd_comp_us) ||
     (output_abs < turn_rev_comp_us)))
{
    /*
     * 5拍一个周期：
     *
     * phase 0 -> kick
     * phase 1 -> kick
     * phase 2 -> 不kick
     * phase 3 -> 不kick
     * phase 4 -> 不kick
     */
    kick =(turn_kick_phase <PID_TURN_KICK_ON_TICKS);

    turn_kick_phase =(uint8_t)((turn_kick_phase + 1U) %PID_TURN_KICK_PERIOD_TICKS);
}
else
{
    /*
     * 当前不需要补偿，
     * 补偿节拍重新从0开始
     */
    turn_kick_phase = 0U;
}


/* 输出给电机 */
PID_ApplyTurn(output, kick);

}

static void PID_ResetEdgePidHistory(void)
{
    edge_error_sum = 0.0f;
    edge_last_error = 0.0f;
    edge_has_last_error = 0U;
}

void PID_ResetEdgeHistory(void)
{
    PID_ResetEdgePidHistory();

    edge_last_position = EDGE_TARGET_POSITION;
    edge_has_last_position = 0U;
}

static uint8_t PID_FindRightEdge(uint8_t white_bits, float *position)
{
    uint8_t left_sensor;
    uint8_t left_mask;
    uint8_t right_mask;

    /* 从右向左找“左白、右黑”的边界 */
    for (left_sensor = 7U; left_sensor > 0U; left_sensor--)
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

    return 0U;//找到返回一，没找到返回0
}

void PID_UpdateRightEdge(uint8_t white_bits)
{
    float position;
    float error;
    float derivative = 0.0f;
    float correction;

    float kp = edge_kp;
    float ki = edge_ki;
    float kd = edge_kd;
    float base_speed = edge_base_speed_us;


    /* --------------------------------------------------
       情况1：边界正好在目标位置
       第4、5路看到白色，第6路为黑色
       -------------------------------------------------- */
    if (((white_bits & 0x18U) == 0x18U) &&
        ((white_bits & 0x04U) == 0U))
    {
        PID_ResetEdgePidHistory();//清除误差和积分

        edge_last_position = EDGE_TARGET_POSITION;
        edge_has_last_position = 1U;

        PID_ApplyForward(base_speed, base_speed);

        return;
    }


    /* --------------------------------------------------
       情况2：尝试寻找右边沿
       -------------------------------------------------- */
    if (PID_FindRightEdge(white_bits, &position) == 0U)
    {
        /*
         * 没找到边界。
         * 这时候清 PID 的积分和微分，
         * 但是保留上一次边界位置。
         */
        PID_ResetEdgePidHistory();

        /*
         * 如果连上一次边界都没有记录，
         * 那就不知道应该往哪个方向找。
         */
        if (edge_has_last_position == 0U)//这个参数相当于一个记录，找到了就给一。一般情况下感觉这个不会发生了
        {
            return;
        }

        /*
         * 根据最后一次边界位置，
         * 判断应该往哪边继续修正。
         */
        error = EDGE_TARGET_POSITION - edge_last_position;

        if (error > 0.0f)
        { 
            correction = kp * error + edge_lost_extra_us;
        }
        else if (error < 0.0f)
        {
            correction = kp * error - edge_lost_extra_us;
        }
        else
        {
            /*
             * 最后位置刚好等于目标，
             * 这时又丢边界，没有方向信息。走着走着线没了，虽然说感觉不会发生，但是也可以写上去
             */
            correction=0;
        }

        correction = PID_Limit(correction, -PID_MAX_US,PID_MAX_US);

        PID_ApplyForward(base_speed - correction,
                         base_speed + correction);

        return;
    }


    /* --------------------------------------------------
       情况3：正常找到右边沿
       -------------------------------------------------- */

    edge_last_position = position;
    edge_has_last_position = 1U;

    error = EDGE_TARGET_POSITION - position;


    /* ---------------- I ---------------- */

    if ((ki > 0.0f) &&
        (error >= -PID_I_ACTIVE_ERROR) &&
        (error <=  PID_I_ACTIVE_ERROR))
    {
        float error_sum_limit;

        error_sum_limit = PID_I_OUTPUT_LIMIT_US / ki;

        edge_error_sum += error;

        edge_error_sum =PID_Limit(edge_error_sum, -error_sum_limit, error_sum_limit);
    }
    else
    {
        edge_error_sum = 0.0f;
    }


    /* ---------------- D ---------------- */

    if (edge_has_last_error != 0U)
    {
        derivative =
            error - edge_last_error;
    }

    edge_last_error = error;
    edge_has_last_error = 1U;


    /* ---------------- PID ---------------- */

    correction = kp * error + ki * edge_error_sum +kd * derivative;


    /* ---------------- 输出 ---------------- */

    correction =PID_Limit(correction, -PID_MAX_US,PID_MAX_US);

    PID_ApplyForward(base_speed - correction,
                     base_speed + correction);
}
/**
 * @brief（简介）方便蓝牙控制调用
 * @param  （参数）
 * @retval （返回值）1说明成功，返回零则失败
 * @note  （说明）用函数去调用，就不用设置宏定义了
 */
uint8_t PID_SetKp(float value)
{
    if (value < 0.0f || value > 1000.0f)
    {
        return 0U;
    }

    pid_kp = value;

    PID_ResetCenterHistory();

    return 1U;
}




uint8_t PID_SetKi(float value)
{
    if (value < 0.0f || value > 1000.0f)
    {
        return 0U;
    }

    pid_ki = value;

    PID_ResetCenterHistory();

    return 1U;
}


uint8_t PID_SetKd(float value)
{
    if (value < 0.0f || value > 1000.0f)
    {
        return 0U;
    }

    pid_kd = value;

    PID_ResetCenterHistory();

    return 1U;
}

uint8_t PID_SetBaseSpeed(float value)
{
    if (value < 0.0f || value > PID_MAX_US)
    {
        return 0U;
    }

    pid_base_speed_us = value;

    return 1U;
}

float PID_GetKp(void)
{
    return pid_kp;
}

void PID_GetParameters(PID_Parameters *parameters)
{
    if (parameters == NULL)
    {
        return;
    }

    parameters->kp = pid_kp;
    parameters->ki = pid_ki;
    parameters->kd = pid_kd;
    parameters->base_speed_us = pid_base_speed_us;
}