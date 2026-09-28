#include "imu.h"
#include "icm42688.h"
#include <math.h>
#include <string.h>

/* 保留参考 IMU.c 的 100 样本静止方差判断和六轴四元数反馈算法。
 * 移除原 TIM3 计时，用 HAL 毫秒差计算真实 dt；不接磁力计。 */
#define CAL_SAMPLES 100U
#define DEG_TO_RAD 0.01745329252f
#define RAD_TO_DEG 57.29577951f

static IMU_State published;
static uint8_t configured;
static uint32_t last_poll, last_sample, last_retry;
static float q[4], feedback[3], offset[3];
static float cal_buffer[CAL_SAMPLES][3];
static double cal_sum[3], cal_square[3]; /* 大角速度后回到静止，避免方差相减精度损失。 */
static uint16_t cal_count, cal_index;

static uint32_t IMU_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static void IMU_Unlock(uint32_t key)
{
    if (key == 0U) __enable_irq();
}

void IMU_GetState(IMU_State *state)
{
    uint32_t key = IMU_Lock();
    *state = published;
    IMU_Unlock(key);
}

uint8_t IMU_GetAngle(float *angle)
{
    IMU_State state;
    if (angle == NULL) return 0U;
    IMU_GetState(&state);
    if (!state.ready || !state.healthy ||
        (uint32_t)(HAL_GetTick() - state.sample_ms) > IMU_STALE_MS)
        return 0U;
    *angle = state.yaw_deg;
    return 1U;
}

float IMU_WrapAngle(float degrees)
{
    while (degrees >= 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

static void IMU_ResetFilter(void)
{
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    memset(feedback, 0, sizeof(feedback));
    memset(offset, 0, sizeof(offset));
    memset(cal_sum, 0, sizeof(cal_sum));
    memset(cal_square, 0, sizeof(cal_square));
    cal_count = cal_index = 0U;
}

static void IMU_Fail(void)
{
    uint32_t key = IMU_Lock();
    published.ready = 0U;
    published.healthy = 0U;
    published.fault_count++; /* 即使故障恢复，控制器也能发现中间曾丢数据。 */
    IMU_Unlock(key);
    configured = 0U;
    last_retry = HAL_GetTick();
    IMU_ResetFilter();
}

void IMU_Init(void)
{
    IMU_ResetFilter();
    memset(&published, 0, sizeof(published));
    configured = ICM42688_Init();
    last_poll = last_sample = last_retry = HAL_GetTick();
    if (!configured) IMU_Fail();
}

static uint8_t IMU_Calibrate(const float gyro[3])
{
    uint8_t i, stable = 1U;
    double mean[3], variance;
    for (i = 0U; i < 3U; i++)
    {
        if (cal_count == CAL_SAMPLES)
        {
            cal_sum[i] -= cal_buffer[cal_index][i];
            cal_square[i] -= (double)cal_buffer[cal_index][i] * cal_buffer[cal_index][i];
        }
        cal_buffer[cal_index][i] = gyro[i];
        cal_sum[i] += gyro[i];
        cal_square[i] += (double)gyro[i] * gyro[i];
    }
    cal_index = (cal_index + 1U) % CAL_SAMPLES;
    if (cal_count < CAL_SAMPLES) cal_count++;
    if (cal_count < CAL_SAMPLES) return 0U;
    for (i = 0U; i < 3U; i++)
    {
        mean[i] = cal_sum[i] / (float)CAL_SAMPLES;
        variance = cal_square[i] / (float)CAL_SAMPLES - mean[i]*mean[i];
        if (variance >= 0.02f || mean[i] <= -3.0f || mean[i] >= 3.0f) stable = 0U;
    }
    if (stable)
    {
        for (i = 0U; i < 3U; i++) offset[i] = (float)mean[i];
    }
    return stable;
}

static uint8_t IMU_UpdateAttitude(const ICM42688_Sample *s, float dt, float *yaw)
{
    float ax = s->accel[0], ay = s->accel[1], az = s->accel[2];
    float gx, gy, gz, vx, vy, vz, ex, ey, ez, norm;
    float a = q[0], b = q[1], c = q[2], d = q[3];
    float half_dt = dt * 0.5f;

    norm = ax*ax + ay*ay + az*az;
    if (!(norm > 0.000001f)) return 0U;
    norm = 1.0f / sqrtf(norm);
    ax *= norm; ay *= norm; az *= norm;
    vx = 2.0f*(b*d - a*c);
    vy = 2.0f*(a*b + c*d);
    vz = a*a - b*b - c*c + d*d;
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;
    feedback[0] += ex * 0.001f * half_dt;
    feedback[1] += ey * 0.001f * half_dt;
    feedback[2] += ez * 0.001f * half_dt;
    gx = (s->gyro[0] - offset[0])*DEG_TO_RAD + 0.5f*ex + feedback[0];
    gy = (s->gyro[1] - offset[1])*DEG_TO_RAD + 0.5f*ey + feedback[1];
    gz = (s->gyro[2] - offset[2])*DEG_TO_RAD + 0.5f*ez + feedback[2];
    q[0] = a + (-b*gx - c*gy - d*gz)*half_dt;
    q[1] = b + (a*gx + c*gz - d*gy)*half_dt;
    q[2] = c + (a*gy - b*gz + d*gx)*half_dt;
    q[3] = d + (a*gz + b*gy - c*gx)*half_dt;
    norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (!(norm > 0.000001f) || !(norm < 100.0f)) return 0U;
    norm = 1.0f / sqrtf(norm);
    q[0] *= norm; q[1] *= norm; q[2] *= norm; q[3] *= norm;
    *yaw = IMU_WrapAngle(IMU_YAW_SIGN * -atan2f(2.0f*(q[1]*q[2] + q[0]*q[3]),
                  1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3])) * RAD_TO_DEG);
    return (*yaw == *yaw);
}

/* 主循环非阻塞轮询 IMU：约每 10 ms 取样；未就绪时用静止样本校准。
 * 读数超时、通信或姿态计算失败时标记故障并定时重试；成功后发布有效 YAW。
 * 阶段切换由 TIM3 中断中的 Stage_Update() 负责，不在这里执行。 */
void IMU_Task(void)
{
    ICM42688_Sample sample;
    IMU_State state;
    uint32_t now = HAL_GetTick(), gap, key;
    int8_t result;
    float yaw;

    if (!configured)
    {
        if ((uint32_t)(now - last_retry) < 1000U) return;
        configured = ICM42688_Init();
        last_poll = last_sample = last_retry = HAL_GetTick();
        return; /* 恢复后重新静止校准，不自动恢复驾驶。 */
    }
    if ((uint32_t)(now - last_poll) < IMU_SAMPLE_PERIOD_MS) return;
    last_poll = now;
    gap = (uint32_t)(now - last_sample);
    if (gap > IMU_STALE_MS) { IMU_Fail(); return; }
    result = ICM42688_Read(&sample);
    if (result < 0) { IMU_Fail(); return; }
    if (result == 0) return;
    last_sample = now;
    IMU_GetState(&state);
    if (!state.ready)
    {
        if (!IMU_Calibrate(sample.gyro)) return;
        state.ready = 1U;
    }
    if (!IMU_UpdateAttitude(&sample, (float)gap * 0.001f, &yaw))
    {
        IMU_Fail();
        return;
    }
    state.yaw_deg = yaw;
    state.sample_ms = now;
    state.healthy = 1U;
    if (gap > state.max_gap_ms) state.max_gap_ms = gap;
    key = IMU_Lock();
    published = state;
    IMU_Unlock(key);
}

