#include "support.h"
#include "imu.h"
#include "icm42688.h"
#include <math.h>
#include <string.h>

static uint32_t tick;
static int8_t read_result = 1;
static uint8_t init_result = 1;
static ICM42688_Sample input;
uint32_t HAL_GetTick(void) { return tick; }
uint8_t ICM42688_Init(void) { return init_result; }
int8_t ICM42688_Read(ICM42688_Sample *sample) { *sample = input; return read_result; }
static void step(void) { tick += 10U; IMU_Task(); }
int main(void)
{
    unsigned i;
    IMU_State state;
    input.accel[2] = 1000.0f;
    input.gyro[2] = 1.0f; /* 静止零偏，校准后应被扣除。 */
    IMU_Init();
    for (i = 0; i < 99; i++) step();
    IMU_GetState(&state); CHECK(!state.ready);
    step(); IMU_GetState(&state); CHECK(state.ready && state.healthy);
    for (i = 0; i < 100; i++) step();
    IMU_GetState(&state); CHECK(fabsf(state.yaw_deg) < 0.01f);
    input.gyro[2] = 91.0f;
    for (i = 0; i < 100; i++) step();
    IMU_GetState(&state); CHECK(fabsf(state.yaw_deg + 90.0f) < 0.1f);
    CHECK(state.max_gap_ms == 10U);
    tick += 20U; IMU_Task(); IMU_GetState(&state);
    CHECK(fabsf(state.yaw_deg + 91.8f) < 0.1f && state.max_gap_ms == 20U);
    read_result = -1; step(); IMU_GetState(&state);
    CHECK(!state.ready && !state.healthy && state.fault_count == 1U);
    read_result = 1; input.gyro[2] = 1; tick += 1000; IMU_Task();
    for (i = 0; i < 100; i++) step();
    IMU_GetState(&state); CHECK(state.ready && state.fault_count == 1U);
    read_result = 0;
    for (i = 0; i < 11; i++) step();
    IMU_GetState(&state); CHECK(!state.ready && state.fault_count == 2U);
    CHECK(IMU_WrapAngle(180) == -180 && IMU_WrapAngle(-181) == 179 && IMU_WrapAngle(359) == -1);
    tick = 0xFFFFFF00U; read_result = 1; input.gyro[2] = 0; IMU_Init();
    for (i = 0; i < 100; i++) step();
    IMU_GetState(&state); CHECK(state.ready && state.healthy); /* 毫秒回绕 */
    /* 持续明显转动不能完成静止校准。 */
    IMU_Init(); input.gyro[2] = 10;
    for (i = 0; i < 150; i++) step();
    IMU_GetState(&state); CHECK(!state.ready);
    return 0;
}
