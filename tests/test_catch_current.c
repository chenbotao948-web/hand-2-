/* Compile current stage.c/usart.c; replace only the hardware and unrelated PID functions. */
#include "support.h"
#include "stage.h"
#include "usart.h"
#include "pid.h"
#include <string.h>

UART_HandleTypeDef huart1;
uint8_t angle_turn_done;
float angle_target_yaw;
static uint32_t tick, left_pwm, right_pwm;
static uint8_t raw_ir = 0x7FU, *rx_byte;
static unsigned center_updates;
static uint8_t center_last_bits;
static char reply[128];
static unsigned reply_len;

uint32_t HAL_GetTick(void) { return tick; }
void PWM_LeftSpeed(uint32_t speed) { left_pwm = speed; }
void PWM_RightSpeed(uint32_t speed) { right_pwm = speed; }
uint8_t IR_Read(void) { return raw_ir; }
uint8_t IMU_GetAngle(float *angle) { *angle = 0.0f; return 1U; }
float IMU_WrapAngle(float angle) { return angle; }
void PID_UpdateCenter(uint8_t bits) { center_last_bits = bits; center_updates++; if (bits != 0U) { left_pwm = 1420U; right_pwm = 1580U; } }
void PID_UpdateAngle(float angle) { (void)angle; }
void PID_UpdateRightEdge(uint8_t bits) { (void)bits; }
void PID_ResetTurnHistory(void) {}
void PID_ResetEdgeHistory(void) {}
uint8_t PID_SetKp(float value) { (void)value; return 1U; }
uint8_t PID_SetKi(float value) { (void)value; return 1U; }
uint8_t PID_SetKd(float value) { (void)value; return 1U; }
uint8_t PID_SetBaseSpeed(float value) { (void)value; return 1U; }
uint8_t PID_SetCenterBaseSpeed(float value) { (void)value; return 1U; }
uint8_t PID_SetEdgeBaseSpeed(float value) { (void)value; return 1U; }
uint8_t PID_SetEKp(float value) { (void)value; return 1U; }
uint8_t PID_SetEKi(float value) { (void)value; return 1U; }
uint8_t PID_SetEKd(float value) { (void)value; return 1U; }
float PID_GetKp(void) { return 0.0f; }
void PID_GetParameters(PID_Parameters *parameters) { memset(parameters, 0, sizeof(*parameters)); }

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t size)
{ (void)uart; (void)size; rx_byte = byte; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)uart; (void)timeout;
    if (reply_len + size < sizeof(reply)) { memcpy(reply + reply_len, data, size); reply_len += size; reply[reply_len] = '\0'; }
    return HAL_OK;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
static void command(const char *line)
{
    reply_len = 0U; reply[0] = '\0';
    while (*line) { *rx_byte = (uint8_t)*line++; HAL_UART_RxCpltCallback(&huart1); }
    UART_Task();
}
static void update_at(uint32_t when) { tick = when; Stage_Update(); }

int main(void)
{
    unsigned before;
    huart1.Instance = USART1;
    tick = 100U; Stage_Init(); UART_Init();
    Stage_SetWaitSeconds(0U);
    command("run\n"); CHECK(Stage_GetState() == STAGE_GO);
    update_at(120U); CHECK(Stage_GetState() == STAGE_CENTER);
    update_at(1199U); CHECK(Stage_GetState() == STAGE_CENTER);
    before = center_updates;
    update_at(1200U);
    CHECK(Stage_GetState() == STAGE_CATCH && left_pwm == 1500U && right_pwm == 1500U);
    CHECK(center_updates == before && Stage_GetElapsedSeconds() == 1U);
    command("STATE?\n"); CHECK(strcmp(reply, "STATE=CATCH\r\n") == 0);
    raw_ir = 0xFFU;
    update_at(4199U); CHECK(Stage_GetState() == STAGE_CATCH && center_updates == before);
    update_at(4200U); CHECK(Stage_GetState() == STAGE_CENTER && center_updates == before + 1U);
    CHECK(center_last_bits == 0x18U && left_pwm == 1420U && right_pwm == 1580U);
    update_at(4220U); CHECK(Stage_GetState() == STAGE_CENTER && center_updates == before + 2U);
    CHECK(center_last_bits == 0U && left_pwm == 1420U && right_pwm == 1580U);
    raw_ir = 0x7FU; update_at(4240U);
    CHECK(Stage_GetState() == STAGE_CENTER && center_last_bits == 0x80U);
    raw_ir = 0xFFU; update_at(4260U);
    CHECK(Stage_GetState() == STAGE_ANGLE); /* Recovery ends on any real white sensor. */
    Stage_SetWaitSeconds(STAGE_DEFAULT_WAIT_SECONDS);

    /* New run resets one-shot state; live edits retain elapsed time. */
    raw_ir = 0x7FU;
    tick = 20000U; command("run\n"); update_at(20020U);
    tick = 21000U; command("CatchTime=3\n"); CHECK(strstr(reply, " OK\r\n") != 0);
    update_at(22000U); CHECK(Stage_GetState() == STAGE_CENTER);
    update_at(22999U); CHECK(Stage_GetState() == STAGE_CENTER);
    update_at(23000U); CHECK(Stage_GetState() == STAGE_CATCH);
    update_at(24000U); command("CatchDuration=2\n"); CHECK(strstr(reply, " OK\r\n") != 0);
    update_at(24999U); CHECK(Stage_GetState() == STAGE_CATCH);
    update_at(25000U); CHECK(Stage_GetState() == STAGE_CENTER);
    command("CatchTime=0\n"); CHECK(strstr(reply, " OK\r\n") != 0);
    update_at(25020U); CHECK(Stage_GetState() == STAGE_CENTER);

    command("CatchDuration=0\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchDuration=6000\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchTime=-1\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchTime=6000\n"); CHECK(strstr(reply, " ERR\r\n") != 0);

    /* GO is not preempted; the expired deadline fires when CENTER is reached. */
    command("CatchTime=5\n"); tick = 30000U; raw_ir = 0xFFU; command("run\n");
    update_at(36000U); CHECK(Stage_GetState() == STAGE_GO);
    raw_ir = 0x7FU; update_at(36020U); CHECK(Stage_GetState() == STAGE_CATCH);
    command("STOP\n"); CHECK(Stage_GetState() == STAGE_STOP && left_pwm == 1500U && right_pwm == 1500U);
    update_at(50000U); CHECK(Stage_GetState() == STAGE_STOP);

    /* A new run during CATCH starts a fresh pickup window. */
    command("CatchTime=0\n"); command("run\n"); CHECK(Stage_GetState() == STAGE_GO);
    update_at(50020U); CHECK(Stage_GetState() == STAGE_CATCH);
    tick = 50040U; command("run\n"); CHECK(Stage_GetState() == STAGE_GO);
    update_at(50060U); CHECK(Stage_GetState() == STAGE_CATCH);

    /* CatchTime accepts integer seconds and up to two decimal places. */
    command("CatchTime=1\n"); CHECK(strcmp(reply, "CatchTime=1 OK\r\n") == 0);
    tick = 60000U; command("run\n"); update_at(60020U);
    update_at(60999U); CHECK(Stage_GetState() == STAGE_CENTER);
    update_at(61000U); CHECK(Stage_GetState() == STAGE_CATCH);

    command("CatchTime=1.5\n"); CHECK(strcmp(reply, "CatchTime=1.5 OK\r\n") == 0);
    tick = 70000U; command("run\n"); update_at(70020U);
    update_at(71499U); CHECK(Stage_GetState() == STAGE_CENTER);
    update_at(71500U); CHECK(Stage_GetState() == STAGE_CATCH);

    command("CatchTime=1.25\n"); CHECK(strcmp(reply, "CatchTime=1.25 OK\r\n") == 0);
    tick = 80000U; command("run\n"); update_at(80020U);
    update_at(81240U); CHECK(Stage_GetState() == STAGE_CENTER);
    update_at(81260U); CHECK(Stage_GetState() == STAGE_CATCH);

    command("CatchTime=0.01\n"); CHECK(strcmp(reply, "CatchTime=0.01 OK\r\n") == 0);
    tick = 90000U; command("run\n"); update_at(90020U);
    CHECK(Stage_GetState() == STAGE_CATCH); /* Next 20 ms control tick. */

    command("CatchTime=5999.00\n"); CHECK(strstr(reply, " OK\r\n") != 0);
    command("CatchTime=5999.01\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchTime=1.234\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchTime=1.\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchTime=.5\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("CatchDuration=1.5\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("Time=1.5\n"); CHECK(strstr(reply, " ERR\r\n") != 0);
    command("SpeedTime=1.5\n"); CHECK(strstr(reply, " ERR\r\n") != 0);

    /* STOP ends recovery, and a fresh run starts without the recovery flag. */
    command("CatchTime=0\n");
    tick = 100000U; command("run\n"); raw_ir = 0xFFU; update_at(100020U);
    raw_ir = 0x7FU; update_at(100040U); CHECK(Stage_GetState() == STAGE_CATCH);
    raw_ir = 0xFFU; update_at(103040U);
    CHECK(Stage_GetState() == STAGE_CENTER && center_last_bits == 0x18U);
    command("STOP\n"); CHECK(Stage_GetState() == STAGE_STOP && left_pwm == 1500U && right_pwm == 1500U);
    command("run\n"); CHECK(Stage_GetState() == STAGE_GO);
    raw_ir = 0x7FU; update_at(103060U); CHECK(Stage_GetState() == STAGE_CATCH);
    return 0;
}
