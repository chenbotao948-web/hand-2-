/* ARMCC 编译真实 pid.c / usart.c，模拟时钟、IMU、红外、电机及 UART 外设。 */
#include "support.h"
#include "pid.h"
#include "usart.h"
#include <string.h>
#include <math.h>

UART_HandleTypeDef huart1;
static uint32_t tick, left_pwm, right_pwm;
static uint8_t raw = 0xFFU, *rx;
static const uint8_t *sending;
static uint16_t sending_size;
static IMU_State sample;
static char output[8192];
static unsigned output_size;

uint32_t HAL_GetTick(void) { return tick; }
void PWM_LeftSpeed(uint32_t value) { left_pwm = value; }
void PWM_RightSpeed(uint32_t value) { right_pwm = value; }
uint8_t IR_Read(void) { return raw; }
void IMU_GetState(IMU_State *state) { *state = sample; }
float IMU_WrapAngle(float value)
{
    while (value >= 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *data, uint16_t size)
{ (void)u; (void)size; rx = data; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, const uint8_t *data, uint16_t size)
{
    (void)u;
    if (sending_size) return HAL_BUSY;
    sending = data; sending_size = size; return HAL_OK;
}
static void drain(void)
{
    while (sending_size)
    {
        memcpy(output + output_size, sending, sending_size);
        output_size += sending_size;
        sending_size = 0;
        HAL_UART_TxCpltCallback(&huart1);
    }
    output[output_size] = 0;
}
static void feed(const char *text)
{
    while (*text) { *rx = (uint8_t)*text++; HAL_UART_RxCpltCallback(&huart1); }
    UART_Task();
}
static void command(const char *text)
{
    drain(); output_size = 0;
    feed(text); drain();
}
static void fresh(float yaw)
{
    sample.yaw_deg = yaw; sample.sample_ms = tick;
    sample.ready = sample.healthy = 1U;
}
int main(void)
{
    PID_AngleStatus status;
    uint32_t saved_left, saved_right;
    unsigned i;
    char too_long[90];
    huart1.Instance = USART1;
    UART_Init(); PID_Init();
    CHECK(!PID_IsRunning() && left_pwm == 1500U && right_pwm == 1500U);
    command("Track=1\n"); CHECK(strstr(output, "ERR") && !PID_IsRunning());
    tick = 100U; fresh(179.0f);
    command("Track=1\r\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_CENTER);
    PID_GetAngleStatus(&status); CHECK(status.initial_deg == 179.0f && status.elapsed_ms == 0U);
    tick += 19999U; fresh(-179.0f); raw = 0xFF;
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    tick++; fresh(-179.0f); raw = 0xE7;
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    saved_left = left_pwm; saved_right = right_pwm;
    raw = 0xFF; fresh(-160.0f);
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    CHECK(left_pwm == saved_left && right_pwm == saved_right);
    fresh(-179.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    CHECK(left_pwm < 1420U && right_pwm < 1580U); /* 左偏时向右纠正 */
    raw = 0x7F; PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_EDGE && PID_GetMode() == PID_MODE_RIGHT_EDGE);
    sample.healthy = 0U; raw = 0xFF; tick += 1000U;
    PID_Update20ms(); CHECK(PID_IsRunning() && PID_GetStage() == PID_STAGE_EDGE);
    fresh(20.0f); command("Track=1\n"); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    PID_GetAngleStatus(&status); CHECK(status.initial_deg == 20.0f && status.elapsed_ms == 0U);
    command("AngleLeft=3\n"); command("AngleRight=7\n");
    tick += 20000U; fresh(23.01f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    fresh(12.99f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    fresh(13.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    raw = 0xFE; PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_EDGE); /* 最右一路也切换 */
    fresh(20); CHECK(PID_Start()); tick += 20000; fresh(23); raw = 0xFF;
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_ANGLE); /* 左界包含等号 */
    raw = 0; PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_EDGE); /* 全白同样切换 */
    command("Track=0\n"); CHECK(!PID_IsRunning() && left_pwm == 1500 && right_pwm == 1500);
    fresh(0); CHECK(PID_Start()); command("Mode=0\n"); tick += 20000;
    sample.healthy = 0; PID_Update20ms(); CHECK(PID_IsRunning() && PID_GetStage() == PID_STAGE_MANUAL);
    fresh(0); CHECK(PID_Start()); sample.healthy = 0; PID_Update20ms();
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT && left_pwm == 1500 && right_pwm == 1500);
    command("Angle?\n"); CHECK(strstr(output, "IMU FAULT STOP") && strstr(output, "Stage=FAULT"));
    fresh(0); PID_Update20ms(); CHECK(!PID_IsRunning());
    CHECK(PID_Start()); tick += 100; PID_Update20ms(); CHECK(PID_IsRunning());
    tick++; PID_Update20ms(); CHECK(!PID_IsRunning());
    fresh(0); CHECK(PID_Start()); sample.fault_count++; PID_Update20ms(); CHECK(!PID_IsRunning());
    tick = 0xFFFFFF00U; raw = 0xFF; fresh(-179); CHECK(PID_Start()); tick += 20000U; fresh(179);
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_ANGLE); /* 计时和航向同时回绕 */
    command("AngleLeft=180\n"); CHECK(strstr(output, "ERR"));
    command("AngleRight=-1\n"); CHECK(strstr(output, "ERR"));
    command("AngleKp=nan\n"); CHECK(strstr(output, "ERR"));
    command("AngleKi=inf\n"); CHECK(strstr(output, "ERR"));
    command("AngleKd=1abc\n"); CHECK(strstr(output, "ERR"));
    command("AngleBase=251\n"); CHECK(strstr(output, "ERR"));
    command("AngleBase=0\n"); CHECK(strstr(output, "OK"));
    command("AngleKp=2.5\n"); CHECK(strstr(output, "OK"));
    command("Angle?\n"); CHECK(strstr(output, "Kp=2.500") && strstr(output, "Right=7.000"));
    CHECK(!PID_SetAngleParameter((PID_AngleParameter)-1, 1.0f));
    memset(too_long, 'a', sizeof(too_long)); too_long[88] = '\n'; too_long[89] = 0;
    command(too_long); CHECK(strstr(output, "ERR"));
    command("AngleLeft=0\n"); CHECK(strstr(output, "OK"));
    /* 持续发送触发 TX 环形缓冲回绕，验证异步输出仍完整。 */
    for (i = 0; i < 20; i++) { command("Angle?\n"); CHECK(strstr(output, "Wait=20000") != 0); }
    /* 堵塞 TX 时停车不能等回复腾空。 */
    UART_SendString(too_long); UART_SendString(too_long); UART_SendString(too_long);
    UART_SendString(too_long); UART_SendString(too_long); UART_SendString(too_long);
    feed("Track=0\n"); CHECK(!PID_IsRunning() && left_pwm == 1500 && right_pwm == 1500);
    drain(); UART_Task(); drain();
    return 0;
}
