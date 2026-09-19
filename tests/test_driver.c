#include "support.h"
#include "icm42688.h"
#include "imu.h"
#include "pid.h"
#include <string.h>
#include <math.h>

SPI_HandleTypeDef hspi2;
static uint8_t registers[128];
static HAL_StatusTypeDef transfer_status = HAL_OK;
static GPIO_PinState cs = GPIO_PIN_SET;
static unsigned cs_error, delay_ms;
static uint32_t tick, left_pwm, right_pwm;
static uint8_t last_read_command, fail_read_address;
static uint16_t last_read_length;
uint32_t HAL_GetTick(void) { return tick; }
uint8_t IR_Read(void) { return 0xFFU; }
void PWM_LeftSpeed(uint32_t value) { left_pwm = value; }
void PWM_RightSpeed(uint32_t value) { right_pwm = value; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port != IMU_CS_GPIO_Port || pin != IMU_CS_Pin) cs_error++;
    cs = state;
}
void HAL_Delay(uint32_t ms) { delay_ms += ms; tick += ms; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, const uint8_t *tx,
                                         uint8_t *rx, uint16_t length, uint32_t timeout)
{
    unsigned i;
    uint8_t address = tx[0] & 0x7FU;
    if (spi != &hspi2 || cs != GPIO_PIN_RESET || timeout > 3U) cs_error++;
    if (transfer_status != HAL_OK) return transfer_status;
    if (tx[0] & 0x80U)
    {
        last_read_command = tx[0];
        last_read_length = length;
        if (address == fail_read_address) return HAL_TIMEOUT;
    }
    memset(rx, 0, length);
    for (i = 1; i < length; i++)
    {
        if (tx[0] & 0x80U) rx[i] = registers[address++];
        else registers[address++] = tx[i];
    }
    return HAL_OK;
}
static void set_axis(unsigned axis, int16_t value)
{
    registers[0x1FU + axis*2U] = (uint8_t)((uint16_t)value >> 8);
    registers[0x20U + axis*2U] = (uint8_t)value;
}
static void step(void)
{
    static uint8_t control_phase;
    tick += 10U;
    IMU_Task();
    control_phase ^= 1U;
    if (control_phase == 0U) PID_Update20ms();
}
int main(void)
{
    ICM42688_Sample sample, saved;
    IMU_State state, before;
    unsigned i;
    const int16_t normal[6] = {-32766, 32767, -1, 0, -12345, 23456};
    registers[0x75] = 0x47;
    CHECK(ICM42688_Init());
    CHECK(registers[0x50] == 0x48 && registers[0x4F] == 0x28 && registers[0x4E] == 0x0F);
    CHECK(delay_ms == 200U && cs == GPIO_PIN_SET && cs_error == 0U);
    CHECK(ICM42688_Read(&sample) == 0);
    registers[0x2D] = 0x08;
    registers[0x1F] = 0x20; registers[0x20] = 0x00; /* +1000mg */
    registers[0x25] = 0xE0; registers[0x26] = 0x00; /* -250deg/s */
    CHECK(ICM42688_Read(&sample) == 1);
    CHECK(last_read_command == 0x9FU && last_read_length == 13U); /* 地址 + 12 数据字节 */
    CHECK(fabsf(sample.accel[0] - 1000) < 0.01f && fabsf(sample.gyro[0] + 250) < 0.01f);
    saved = sample;
    registers[0x2D] = 0;
    CHECK(ICM42688_Read(&sample) == 0 && memcmp(&sample, &saved, sizeof(sample)) == 0);
    registers[0x2D] = 0x08;
    /* 六轴逐一注入哨兵值，尤其最后一个轴异常时也不能覆盖前面已解析的轴。 */
    for (i = 0; i < 6; i++)
    {
        memset(&registers[0x1F], 0, 12);
        set_axis(i, -32768);
        CHECK(ICM42688_Read(&sample) == 0);
        CHECK(memcmp(&sample, &saved, sizeof(sample)) == 0 && cs == GPIO_PIN_SET);
    }
    memset(&registers[0x1F], 0, 12);
    for (i = 0; i < 3; i++) set_axis(i, -1);
    CHECK(ICM42688_Read(&sample) == 0 && memcmp(&sample, &saved, sizeof(sample)) == 0);
    for (i = 0; i < 6; i++) set_axis(i, normal[i]);
    CHECK(ICM42688_Read(&sample) == 1);
    for (i = 0; i < 3; i++)
    {
        CHECK(sample.accel[i] == (float)normal[i] * (4000.0f / 32768.0f));
        CHECK(sample.gyro[i] == (float)normal[i+3] * (1000.0f / 32768.0f));
    }
    for (i = 0; i < 3; i++) { set_axis(i, 0); set_axis(i+3, -1); }
    CHECK(ICM42688_Read(&sample) == 1); /* 不把正常零值或角速度全 -1 错判为加速度异常 */
    saved = sample;
    fail_read_address = 0x1F;
    CHECK(ICM42688_Read(&sample) == -1 && memcmp(&sample, &saved, sizeof(sample)) == 0);
    CHECK(cs == GPIO_PIN_SET);
    fail_read_address = 0;
    transfer_status = HAL_TIMEOUT;
    CHECK(ICM42688_Read(&sample) == -1 && cs == GPIO_PIN_SET);
    CHECK(memcmp(&sample, &saved, sizeof(sample)) == 0);
    CHECK(!ICM42688_Init() && cs == GPIO_PIN_SET);
    transfer_status = HAL_OK; registers[0x75] = 0xFF;
    CHECK(ICM42688_Read(&sample) == -1 && !ICM42688_Init());
    CHECK(cs_error == 0U);

    /* 真实 driver -> imu -> pid 联合验证：异常帧不能冒充有效样本。 */
    registers[0x75] = 0x47;
    memset(&registers[0x1F], 0, 12);
    set_axis(2, 8192); /* 静止，Z 轴 1g */
    PID_Init(); IMU_Init();
    for (i = 0; i < 99; i++) step();
    IMU_GetState(&state); CHECK(!state.ready);
    set_axis(5, -32768); step();
    IMU_GetState(&state); CHECK(!state.ready && !PID_Start());
    set_axis(5, 0); step();
    IMU_GetState(&state); CHECK(state.ready && state.healthy && PID_Start());
    before = state;
    set_axis(5, -32768); step();
    IMU_GetState(&state);
    CHECK(state.sample_ms == before.sample_ms && state.yaw_deg == before.yaw_deg);
    CHECK(PID_IsRunning() && state.fault_count == before.fault_count);
    set_axis(5, 0); step();
    IMU_GetState(&state);
    CHECK(state.sample_ms == tick && state.ready && PID_IsRunning());
    before = state;
    for (i = 0; i < 3; i++) set_axis(i, -1);
    for (i = 0; i < 10; i++) step();
    IMU_GetState(&state);
    CHECK(state.sample_ms == before.sample_ms && state.yaw_deg == before.yaw_deg);
    CHECK(PID_IsRunning()); /* 恰好 100ms 尚未超过阈值 */
    step(); IMU_GetState(&state);
    CHECK(!state.ready && !state.healthy && state.fault_count == before.fault_count + 1U);
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT);
    CHECK(left_pwm == 1500U && right_pwm == 1500U && PID_TakeImuFault());
    memset(&registers[0x1F], 0, 12); set_axis(2, 8192);
    tick += 1000U; IMU_Task();
    for (i = 0; i < 100; i++) step();
    IMU_GetState(&state);
    CHECK(state.ready && state.healthy && !PID_IsRunning());
    CHECK(PID_Start()); /* 必须重新启动，恢复不自动行驶 */
    return 0;
}
