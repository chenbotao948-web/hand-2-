/* ARMCC 编译真实 pid.c / usart.c，模拟时钟、IMU、红外、电机及 UART 外设。 */
#include "support.h"
#include "pid.h"
#include "usart.h"
#include <string.h>
#include <math.h>

UART_HandleTypeDef huart1;
static uint32_t tick, left_pwm, right_pwm;
static uint8_t raw = 0xFFU, *rx;
static unsigned ir_reads;
static const uint8_t *sending;
static uint16_t sending_size;
static IMU_State sample;
static char output[8192];
static unsigned output_size;

uint32_t HAL_GetTick(void) { return tick; }
void PWM_LeftSpeed(uint32_t value) { left_pwm = value; }
void PWM_RightSpeed(uint32_t value) { right_pwm = value; }
uint8_t IR_Read(void) { ir_reads++; return raw; }
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
static void advance(float yaw, unsigned count)
{
    while (count--) { tick += 20U; fresh(yaw); PID_Update20ms(); }
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
    PID_GetAngleStatus(&status);
    CHECK(status.turn_tol_deg == 10.0f && status.wait_ms == 0U && status.turn_kp == 3.0f);
    CHECK(status.turn_fwd_comp_us == 40.0f && status.turn_rev_comp_us == 40.0f);
    command("AngleWait=20000\n");
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
    fresh(-179.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    CHECK(left_pwm == 1500U && right_pwm == 1500U);
    advance(-179.0f, 3); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    CHECK(left_pwm == 1500U && right_pwm == 1500U);
    advance(-165.0f, 1);
    CHECK(left_pwm > 1420U && right_pwm > 1580U); /* Yaw 大于目标时向左纠正。 */
    raw = 0x7F; PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_EDGE && PID_GetMode() == PID_MODE_RIGHT_EDGE);
    sample.healthy = 0U; raw = 0xFF; tick += 1000U;
    PID_Update20ms(); CHECK(PID_IsRunning() && PID_GetStage() == PID_STAGE_EDGE);
    fresh(20.0f); command("Track=1\n"); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    PID_GetAngleStatus(&status); CHECK(status.initial_deg == 20.0f && status.elapsed_ms == 0U);
    command("AngleLeft=3\n"); command("AngleRight=7\n");
    tick += 20000U; fresh(23.01f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    fresh(12.99f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    fresh(13.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    advance(13.0f, 3); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    raw = 0xFE; PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_EDGE); /* 最右一路也切换 */
    fresh(20); CHECK(PID_Start()); tick += 20000; fresh(23); raw = 0xFF;
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN); /* 左界包含等号 */
    advance(23.0f, 3); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
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
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN); /* 计时和航向同时回绕 */
    command("AngleLeft=180\n"); CHECK(strstr(output, "ERR"));
    command("AngleRight=-1\n"); CHECK(strstr(output, "ERR"));
    command("AngleKp=nan\n"); CHECK(strstr(output, "ERR"));
    command("AngleKi=inf\n"); CHECK(strstr(output, "ERR"));
    command("AngleKd=1abc\n"); CHECK(strstr(output, "ERR"));
    command("AngleBase=251\n"); CHECK(strstr(output, "ERR"));
    command("AngleBase=0\n"); CHECK(strstr(output, "OK"));
    command("AngleKp=2.5\n"); CHECK(strstr(output, "OK"));
    command("Angle?\n"); CHECK(strstr(output, "Kp=2.500") && strstr(output, "Right=7.000"));
    command("AngleWait=-1\n"); CHECK(strstr(output, "ERR"));
    command("AngleWait=1abc\n"); CHECK(strstr(output, "ERR"));
    command("AngleWait=2147483648\n"); CHECK(strstr(output, "ERR"));
    command("AngleWait=500\n"); CHECK(strstr(output, "OK"));
    command("Angle?\n"); CHECK(strstr(output, "Wait=500"));
    fresh(0.0f); raw = 0xFFU; CHECK(PID_Start());
    tick += 499U; fresh(0.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    tick++; fresh(0.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    command("Track=0\n");
    CHECK(!PID_SetAngleParameter((PID_AngleParameter)-1, 1.0f));
    memset(too_long, 'a', sizeof(too_long)); too_long[88] = '\n'; too_long[89] = 0;
    command(too_long); CHECK(strstr(output, "ERR"));
    command("AngleLeft=0\n"); CHECK(strstr(output, "OK"));
    /* 持续发送触发 TX 环形缓冲回绕，验证异步输出仍完整。 */
    for (i = 0; i < 20; i++) { command("Angle?\n"); CHECK(strstr(output, "Wait=500") != 0); }
    /* 堵塞 TX 时停车不能等回复腾空。 */
    UART_SendString(too_long); UART_SendString(too_long); UART_SendString(too_long);
    UART_SendString(too_long); UART_SendString(too_long); UART_SendString(too_long);
    feed("Track=0\n"); CHECK(!PID_IsRunning() && left_pwm == 1500 && right_pwm == 1500);
    drain(); UART_Task(); drain();

    /* 独立控制参数显式使用测试值，避免依赖上电默认。 */
    command("TurnKp=2\n"); command("TurnTol=2\n");
    command("TurnFwdComp=20\n"); command("TurnRevComp=30\n");
    /* 原地转向可被循迹命令接管；命令左转为正，实车 IMU 左转为负。 */
    sample.ready = 0U;
    command("TurnTo=-30\n"); CHECK(strstr(output, "ERR") && !PID_IsRunning());
    fresh(0.0f);
    command("Turn=30\n"); CHECK(strstr(output, "ERR") && !PID_IsRunning());
    command("TurnTo=-30\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_TURN);
    saved_left = ir_reads;
    PID_Update20ms();
    CHECK(left_pwm < 1500U && right_pwm < 1500U && ir_reads == saved_left);
    command("Track=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_CENTER);
    command("Mode=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_MANUAL);
    command("TurnTo=-30\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_TURN);
    command("Angle?\n"); CHECK(strstr(output, "Target=30.000 Error=30.000 Hold=0"));
    for (i = 0; i < 2; i++)
    {
        tick += 20U; fresh(29.0f); PID_Update20ms();
        CHECK(left_pwm == 1500U && right_pwm == 1500U);
    }
    PID_GetAngleStatus(&status); CHECK(!status.turn_holding);
    tick += 20U; fresh(29.0f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(status.turn_holding && PID_IsRunning());
    tick += 20U; fresh(28.0f); PID_Update20ms();
    PID_GetAngleStatus(&status);
    CHECK(status.turn_holding && left_pwm == 1500U && right_pwm == 1500U);
    tick += 20U; fresh(27.0f); PID_Update20ms();
    PID_GetAngleStatus(&status);
    CHECK(!status.turn_holding && left_pwm < 1500U && right_pwm < 1500U);
    command("LeftSpeed=1400\n"); CHECK(strstr(output, "ERR"));
    command("Track=0\n"); CHECK(!PID_IsRunning() && left_pwm == 1500U && right_pwm == 1500U);
    fresh(0.0f); command("TurnTo=30\n"); PID_Update20ms();
    PID_GetAngleStatus(&status);
    CHECK(status.turn_target_deg == -30.0f && status.turn_error_deg == -30.0f &&
          left_pwm > 1500U && right_pwm > 1500U); /* 命令左转 30°。 */
    command("Track=0\n");

    /* 容差运行中生效；两侧同阈值，到位连续计数在修改时重新开始。 */
    command("TurnTol=0.5\n"); CHECK(strstr(output, "OK"));
    command("Angle?\n"); CHECK(strstr(output, "Tol=0.500"));
    fresh(0.0f); command("TurnTo=-30\n");
    tick += 20U; fresh(29.0f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(!status.turn_holding && left_pwm < 1500U);
    command("TurnTol=2\n"); CHECK(strstr(output, "OK"));
    tick += 20U; fresh(29.0f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(!status.turn_holding && left_pwm == 1500U);
    command("TurnTol=0.5\n");
    for (i = 0; i < 2; i++)
    {
        tick += 20U; fresh(29.5f); PID_Update20ms();
        PID_GetAngleStatus(&status); CHECK(!status.turn_holding);
    }
    tick += 20U; fresh(29.5f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(status.turn_holding);
    command("TurnTol=0.1\n"); CHECK(strstr(output, "OK"));
    tick += 20U; fresh(29.5f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(!status.turn_holding && left_pwm < 1500U);
    command("TurnTol=10\n"); CHECK(strstr(output, "OK"));
    for (i = 0; i < 2; i++)
    {
        tick += 20U; fresh(20.0f); PID_Update20ms();
    }
    PID_GetAngleStatus(&status); CHECK(!status.turn_holding && left_pwm == 1500U);
    command("TurnTol=10\n"); /* 同值重新设置也清零连续计数。 */
    tick += 20U; fresh(20.0f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(!status.turn_holding);
    for (i = 0; i < 2; i++) { tick += 20U; fresh(20.0f); PID_Update20ms(); }
    PID_GetAngleStatus(&status); CHECK(status.turn_holding);
    command("TurnTol=0.09\n"); CHECK(strstr(output, "ERR"));
    command("TurnTol=10.01\n"); CHECK(strstr(output, "ERR"));
    command("TurnTol=nan\n"); CHECK(strstr(output, "ERR"));
    command("TurnTol=inf\n"); CHECK(strstr(output, "ERR"));
    command("TurnTol=abc\n"); CHECK(strstr(output, "ERR"));
    command("TurnTol=\n"); CHECK(strstr(output, "ERR"));
    PID_GetAngleStatus(&status); CHECK(status.turn_tol_deg == 10.0f);
    command("Track=0\n"); command("TurnTol=2\n");
    fresh(25.0f); command("TurnTo=5\n"); CHECK(PID_GetStage() == PID_STAGE_TURN);
    PID_Update20ms(); CHECK(left_pwm == 1560U && right_pwm == 1560U);
    command("Track=0\n");

    /* 低输出短脉冲分别叠加前进和后退补偿，随后恢复 PID 原输出。 */
    command("TurnKp=0.5\n"); command("TurnKd=0\n");
    fresh(0.0f); command("TurnTo=-30\n"); PID_Update20ms();
    CHECK(left_pwm == 1465U && right_pwm == 1455U);
    tick += 20U; fresh(25.0f); PID_Update20ms();
    CHECK(left_pwm == 1478U && right_pwm == 1468U);
    tick += 20U; fresh(32.5f); PID_Update20ms();
    CHECK(left_pwm == 1501U && right_pwm == 1501U);
    command("Track=0\n");
    command("TurnMin=0\n");
    fresh(0.0f); command("TurnTo=-30\n"); PID_Update20ms();
    CHECK(left_pwm == 1485U && right_pwm == 1455U);
    command("Track=0\n");

    command("TurnMin=20\n"); command("TurnKp=2\n");
    fresh(-39.215f); command("TurnTo=30\n");
    for (i = 0; i < 2U; i++)
    {
        tick += 20U; fresh(-39.215f); PID_Update20ms();
        CHECK(left_pwm == 1462U && right_pwm == 1452U);
    }
    for (i = 0; i < 3U; i++)
    {
        tick += 20U; fresh(-39.215f); PID_Update20ms();
        CHECK(left_pwm == 1482U && right_pwm == 1482U);
    }
    tick += 20U; fresh(-39.215f); PID_Update20ms();
    CHECK(left_pwm == 1462U && right_pwm == 1452U);
    tick += 20U; fresh(-28.0f); PID_Update20ms();
    CHECK(left_pwm == 1500U && right_pwm == 1500U);
    command("Track=0\n");
    fresh(39.215f); command("TurnTo=-30\n"); PID_Update20ms();
    CHECK(left_pwm == 1548U && right_pwm == 1538U);
    command("Track=0\n");
    command("TurnFwdComp=25\n"); command("TurnRevComp=35\n");
    fresh(-39.215f); command("TurnTo=30\n"); PID_Update20ms();
    CHECK(left_pwm == 1457U && right_pwm == 1447U);
    command("Track=0\n");
    command("TurnFwdComp=20\n"); command("TurnRevComp=30\n");

    /* D 项需要能真正减速或反向制动，不能再被误差符号强制覆盖。 */
    command("TurnMin=20\n"); command("TurnKp=2\n"); command("TurnKd=30\n");
    fresh(0.0f); command("TurnTo=-30\n");
    tick += 20U; fresh(18.0f); PID_Update20ms();
    CHECK(left_pwm == 1456U && right_pwm == 1446U);
    tick += 20U; fresh(19.0f); PID_Update20ms();
    CHECK(left_pwm == 1508U && right_pwm == 1508U);
    command("Track=0\n"); command("TurnKd=0\n");

    fresh(179.0f); command("TurnTo=176\n");
    PID_GetAngleStatus(&status);
    CHECK(status.turn_valid && status.turn_target_deg == -176.0f && status.turn_error_deg == 5.0f);
    tick += 20U; fresh(-178.0f); PID_Update20ms();
    PID_GetAngleStatus(&status); CHECK(status.turn_error_deg == 2.0f);
    command("Track=0\n");
    fresh(20.0f); command("TurnTo=-90\n");
    PID_GetAngleStatus(&status);
    CHECK(status.turn_target_deg == 90.0f && status.turn_error_deg == 70.0f);
    PID_Update20ms(); CHECK(left_pwm < 1500U && right_pwm < 1500U);
    command("TurnTo=0\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_TURN);
    command("Track=0\n");
    /* 用户现场：左转 90° 对应 Yaw≈-90°，10° 容差内必须停车。 */
    command("TurnTol=10\n");
    fresh(-88.645f); command("TurnTo=90\n");
    PID_GetAngleStatus(&status);
    CHECK(status.turn_target_deg == -90.0f && status.turn_error_deg < -1.3f &&
          status.turn_error_deg > -1.4f && status.turn_tol_deg == 10.0f);
    PID_Update20ms(); CHECK(left_pwm == 1500U && right_pwm == 1500U);
    for (i = 0; i < 2; i++)
    {
        tick += 20U; fresh(-90.012f); PID_Update20ms();
    }
    PID_GetAngleStatus(&status); CHECK(status.turn_holding && !status.turn_timeout);
    command("Angle?\n"); CHECK(strstr(output, "Target=-90.000") && strstr(output, "Tol=10.000"));
    command("Track=0\n"); command("TurnTol=2\n");
    fresh(179.0f); command("TurnTo=179\n");
    PID_GetAngleStatus(&status);
    CHECK(status.turn_target_deg == -179.0f && status.turn_error_deg == 2.0f);
    command("Track=0\n");
    fresh(-179.0f); command("TurnTo=-179\n");
    PID_GetAngleStatus(&status);
    CHECK(status.turn_target_deg == 179.0f && status.turn_error_deg == -2.0f);
    command("Track=0\n");
    command("TurnTo=-180\n"); CHECK(strstr(output, "OK"));
    PID_GetAngleStatus(&status); CHECK(status.turn_target_deg == -180.0f);
    command("Track=0\n");
    command("TurnTo=180\n"); CHECK(strstr(output, "ERR"));
    command("TurnTo=-181\n"); CHECK(strstr(output, "ERR"));
    command("TurnTo=nan\n"); CHECK(strstr(output, "ERR"));
    command("TurnTo=inf\n"); CHECK(strstr(output, "ERR"));
    command("TurnTo=\n"); CHECK(strstr(output, "ERR"));
    command("Turn=90\n"); CHECK(strstr(output, "ERR"));
    command("TurnMin=81\n"); CHECK(strstr(output, "ERR"));
    command("TurnMax=19\n"); CHECK(strstr(output, "ERR"));
    command("TurnRevMin=81\n"); CHECK(strstr(output, "ERR"));
    command("TurnRevMin=nan\n"); CHECK(strstr(output, "ERR"));
    command("TurnRevMin=30\n"); CHECK(strstr(output, "OK"));
    command("Angle?\n"); CHECK(strstr(output, "FwdComp=20.000") && strstr(output, "RevComp=30.000"));
    command("TurnKp=2.5\n"); CHECK(strstr(output, "OK"));
    command("TurnKi=0\n"); CHECK(strstr(output, "OK"));
    command("TurnKd=0.5\n"); CHECK(strstr(output, "OK"));
    command("Angle?\n"); CHECK(strstr(output, "TURN Kp=2.500 Ki=0.000 Kd=0.500"));

    fresh(0.0f); command("TurnTo=-90\n");
    tick += 101U; PID_Update20ms();
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT &&
          left_pwm == 1500U && right_pwm == 1500U);
    command("Angle?\n"); CHECK(strstr(output, "IMU FAULT STOP"));
    fresh(0.0f); command("TurnTo=-90\n");
    sample.fault_count++; PID_Update20ms();
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT);
    fresh(0.0f); command("TurnTo=-90\n");
    tick += 10000U; fresh(0.0f); PID_Update20ms();
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT &&
          left_pwm == 1500U && right_pwm == 1500U);
    command("Angle?\n"); CHECK(strstr(output, "TURN TIMEOUT STOP") && strstr(output, "Timeout=1"));
    fresh(0.0f); command("TurnTo=0\n");
    for (i = 0; i < 3; i++) { tick += 20U; fresh(0.0f); PID_Update20ms(); }
    PID_GetAngleStatus(&status); CHECK(status.turn_holding);
    sample.healthy = 0U; PID_Update20ms();
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT &&
          left_pwm == 1500U && right_pwm == 1500U);

    /* angle=1 的对准阶段复用同一短脉冲，进入容差立即停车。 */
    PID_Init(); fresh(-39.215f); command("TurnTol=2\n");
    command("TurnKp=2\n"); command("TurnKd=0\n");
    command("AngleTarget=30\n"); command("angle=1\n");
    PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN &&
          left_pwm == 1462U && right_pwm == 1452U);
    tick += 20U; fresh(-29.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN &&
          left_pwm == 1500U && right_pwm == 1500U);
    command("angle=0\n"); CHECK(!PID_IsRunning());

    /* 直行目标只保存，启动后必须先原地对准三个控制拍。 */
    PID_Init(); fresh(0.0f); command("TurnTol=2\n");
    command("angle=1\n"); CHECK(strstr(output, "ERR") && !PID_IsRunning());
    command("Angle?\n"); CHECK(strstr(output, "STRAIGHT Target=NA Error=NA"));
    command("AngleTarget=180\n"); CHECK(strstr(output, "ERR"));
    command("AngleTarget=-181\n"); CHECK(strstr(output, "ERR"));
    command("AngleTarget=nan\n"); CHECK(strstr(output, "ERR"));
    command("AngleTarget=inf\n"); CHECK(strstr(output, "ERR"));
    command("AngleTarget=\n"); CHECK(strstr(output, "ERR"));
    command("TurnKp=2\n"); command("TurnKd=0\n");
    command("AngleKp=2\n"); command("AngleKi=0\n");
    command("AngleKd=0\n"); command("AngleBase=80\n");
    command("AngleTarget=30\n");
    CHECK(strstr(output, "OK") && !PID_IsRunning() &&
          left_pwm == 1500U && right_pwm == 1500U);
    command("Angle?\n"); CHECK(strstr(output, "STRAIGHT Target=30.000 Error=30.000"));
    command("angle=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    saved_left = ir_reads;
    PID_Update20ms(); CHECK(left_pwm > 1500U && right_pwm > 1500U && ir_reads == saved_left);
    command("angle=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    tick += 20U; fresh(-29.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN && left_pwm == 1500U);
    tick += 20U; fresh(-29.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN && right_pwm == 1500U);
    tick += 20U; fresh(-29.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT && left_pwm == 1500U && right_pwm == 1500U);
    tick += 20U; fresh(-29.0f); PID_Update20ms();
    CHECK(left_pwm == 1420U && right_pwm == 1580U && ir_reads == saved_left);
    tick += 20U; fresh(-25.0f); PID_Update20ms();
    CHECK(left_pwm == 1426U && right_pwm == 1586U);
    tick += 20U; fresh(-35.0f); PID_Update20ms();
    CHECK(left_pwm == 1414U && right_pwm == 1574U);
    command("Angle?\n"); CHECK(strstr(output, "Stage=STRAIGHT") &&
                                 strstr(output, "STRAIGHT Target=30.000 Error=-5.000"));
    command("angle=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_STRAIGHT);
    command("Track=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_CENTER);
    command("Mode=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_MANUAL);
    command("angle=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    advance(-29.0f, 3); CHECK(PID_GetStage() == PID_STAGE_STRAIGHT);
    command("TurnTol=10\n"); tick += 20U; fresh(-35.0f); PID_Update20ms();
    CHECK(left_pwm == 1420U && right_pwm == 1580U);
    command("TurnTol=2\n");

    /* 运行中换目标先停车，重新对准后自动恢复前进。 */
    command("AngleTarget=-30\n");
    CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN &&
          left_pwm == 1500U && right_pwm == 1500U);
    PID_Update20ms(); CHECK(left_pwm < 1500U && right_pwm < 1500U);
    tick += 20U; fresh(30.0f); PID_Update20ms();
    tick += 20U; fresh(30.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    command("AngleTarget=-30\n"); CHECK(left_pwm == 1500U && right_pwm == 1500U);
    for (i = 0; i < 2; i++) { tick += 20U; fresh(30.0f); PID_Update20ms(); }
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    tick += 20U; fresh(30.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT && left_pwm == 1500U);
    tick += 20U; fresh(30.0f); PID_Update20ms();
    CHECK(left_pwm == 1420U && right_pwm == 1580U);
    command("angle=0\n"); CHECK(strstr(output, "OK") && !PID_IsRunning() &&
                                  left_pwm == 1500U && right_pwm == 1500U);
    command("angle=1\n"); CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    UART_SendString(too_long); UART_SendString(too_long); UART_SendString(too_long);
    UART_SendString(too_long); UART_SendString(too_long); UART_SendString(too_long);
    feed("angle=0\n"); CHECK(!PID_IsRunning() && left_pwm == 1500U && right_pwm == 1500U);
    drain(); UART_Task(); drain();
    command("angle=1\n"); CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    command("Track=0\n"); CHECK(!PID_IsRunning() && left_pwm == 1500U && right_pwm == 1500U);

    /* 跨越 180° 边界走最短路径；独立转向时改直行目标只保存。 */
    fresh(179.0f); command("AngleTarget=176\n"); command("angle=1\n");
    PID_GetAngleStatus(&status);
    CHECK(status.turn_error_deg == 5.0f && status.straight_error_deg == -5.0f);
    PID_Update20ms(); CHECK(left_pwm < 1500U && right_pwm < 1500U);
    command("Track=0\n");
    fresh(0.0f); command("TurnTo=0\n");
    command("AngleTarget=-180\n");
    CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_TURN);
    command("angle=1\n"); CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    command("Track=0\n");

    /* 对准超时、直行 IMU 数据过期都要停车，不自行重启。 */
    fresh(0.0f); command("angle=1\n");
    tick += 10000U; fresh(0.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_FAULT && !PID_IsRunning() &&
          left_pwm == 1500U && right_pwm == 1500U);
    command("Angle?\n"); CHECK(strstr(output, "TURN TIMEOUT STOP"));
    command("AngleTarget=0\n"); fresh(0.0f); command("angle=1\n");
    for (i = 0; i < 3; i++) { tick += 20U; fresh(0.0f); PID_Update20ms(); }
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT);
    tick += 101U; PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_FAULT && !PID_IsRunning() &&
          left_pwm == 1500U && right_pwm == 1500U);
    PID_Init(); fresh(0.0f); command("TurnTol=2\n");
    command("AngleTarget=30\n"); command("angle=1\n");
    sample.fault_count++; PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_FAULT && !PID_IsRunning() &&
          left_pwm == 1500U && right_pwm == 1500U);
    return 0;
}
