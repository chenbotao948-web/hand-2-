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
    PID_Parameters params;
    unsigned reads, i;
    uint32_t elapsed;
    char payload[90], queued[256];
    huart1.Instance = USART1;
    UART_Init(); PID_Init(); tick = 100U; fresh(17.0f);
    PID_GetAngleStatus(&status);
    CHECK(status.wait_ms == 0U && status.turn_tol_deg == 10.0f && status.turn_kp == 3.0f);
    CHECK(status.turn_fwd_comp_us == 40.0f && status.turn_rev_comp_us == 40.0f);
    CHECK(status.base_us == 80.0f && status.kp == 2.0f && status.ki == 0.0f && status.kd == 0.0f);
    command("AngleTarget=120\n"); command("Track=1\n");
    CHECK(strstr(output, "OK") && PID_GetStage() == PID_STAGE_CENTER);
    raw = 0xE7; advance(17.0f, 1); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    raw = 0xFF; advance(28.0f, 1); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    advance(6.0f, 1); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    advance(27.0f, 1); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    CHECK(left_pwm == 1500U && right_pwm == 1500U);
    PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 17.0f && status.active_target_deg == 17.0f && status.active_error_deg == -10.0f);
    CHECK(status.straight_target_deg == 120.0f); /* 自动目标不得覆盖独立目标。 */
    command("Angle?\n");
    CHECK(strstr(output,"Stage=AUTO_ALIGN") && strstr(output,"ACTIVE TargetYaw=17.000 Error=-10.000"));
    CHECK(strstr(output,"STRAIGHT Target=120.000"));
    reads = ir_reads; raw = 0x00;
    advance(27.0f, 2); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN && ir_reads == reads);
    advance(29.0f, 1); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN && left_pwm > 1500U);
    advance(27.0f, 2); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    advance(27.0f, 1); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    CHECK(left_pwm == 1500U && right_pwm == 1500U && ir_reads == reads);
    advance(27.0f, 1); CHECK(PID_GetStage() == PID_STAGE_EDGE && PID_GetMode() == PID_MODE_RIGHT_EDGE);
    CHECK(ir_reads == reads + 1U); /* 到位后下一拍才读线并交给 EDGE。 */

    command("Stage=ANGLE\n"); CHECK(strstr(output,"OK") && PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    command("AngleTarget=-70\n"); PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 17.0f && status.active_target_deg == 17.0f && status.straight_target_deg == -70.0f);
    raw = 0xFF; advance(17.0f, 3); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    elapsed = status.elapsed_ms;
    command("Stage=ANGLE\n"); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    advance(17.0f, 1); CHECK(left_pwm == 1420U && right_pwm == 1580U);
    advance(30.0f, 1); CHECK(left_pwm == 1426U && right_pwm == 1586U);
    advance(4.0f, 1); CHECK(left_pwm == 1414U && right_pwm == 1574U);
    command("Angle?\n"); CHECK(strstr(output,"ACTIVE TargetYaw=17.000 Error=13.000"));
    /* 自动直行修改容差也必须清除旧的积分和差分。 */
    command("AngleKi=1\n"); command("AngleKd=1\n");
    advance(30.0f,1); CHECK(left_pwm == 1429U && right_pwm == 1589U);
    command("TurnTol=5\n"); advance(30.0f,1);
    CHECK(left_pwm == 1444U && right_pwm == 1604U);
    command("AngleKi=0\n"); command("AngleKd=0\n"); command("TurnTol=10\n");
    raw = 0x7F; advance(4.0f, 1); CHECK(PID_GetStage() == PID_STAGE_EDGE);
    PID_GetAngleStatus(&status); CHECK(!status.active_valid && status.initial_deg == 17.0f && status.elapsed_ms > elapsed);
    command("Stage=CENTER\n"); CHECK(PID_GetStage() == PID_STAGE_CENTER && PID_GetMode() == PID_MODE_CENTER);
    command("Stage=bad\n"); CHECK(strstr(output,"ERR") && PID_GetStage() == PID_STAGE_CENTER);
    CHECK(!PID_SelectStage((PID_Stage)-1) && !PID_SelectStage(PID_STAGE_FAULT));

    /* 超过20秒再切对准，不得套用起跑时间；重复选阶段不得延长超时。 */
    tick += 25000U; fresh(90.0f); command("Stage=ANGLE\n");
    PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 17.0f && status.elapsed_ms >= 25000U && status.active_target_deg == 17.0f);
    elapsed = status.elapsed_ms;
    PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN && !status.turn_timeout);
    tick += 9999U; fresh(90.0f); PID_Update20ms(); CHECK(PID_IsRunning());
    command("Stage=ANGLE\n"); PID_GetAngleStatus(&status);
    CHECK(status.elapsed_ms == elapsed + 9999U && status.initial_deg == 17.0f);
    tick++; fresh(90.0f); PID_Update20ms();
    CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_FAULT && left_pwm == 1500U && right_pwm == 1500U);
    command("Angle?\n"); CHECK(strstr(output,"TURN TIMEOUT STOP") && strstr(output,"Timeout=1"));
    command("Stage=ANGLE\n"); PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 90.0f && status.elapsed_ms == 0U && status.active_target_deg == 90.0f);
    command("Stage=CENTER\n"); advance(70.0f,1); /* 与90度差20度，不能自动切换。 */
    CHECK(PID_GetStage() == PID_STAGE_CENTER);
    command("Stage=CENTER\n"); PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 90.0f && status.elapsed_ms == 20U);

    /* 等待时间和角度同时跨回绕；单侧容差也按现有 delta 约定生效。 */
    command("Stage=STOP\n"); tick = 0xFFFFFF00U; fresh(179.0f); raw = 0xFF;
    command("AngleWait=500\n"); command("AngleLeft=3\n"); command("AngleRight=7\n");
    command("Track=1\n"); tick += 499U; fresh(-179.0f); PID_Update20ms();
    CHECK(PID_GetStage() == PID_STAGE_CENTER);
    tick++; fresh(-177.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    fresh(171.9f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_CENTER);
    fresh(172.0f); PID_Update20ms(); CHECK(PID_GetStage() == PID_STAGE_AUTO_ALIGN);
    PID_GetAngleStatus(&status); CHECK(status.active_target_deg == 179.0f && status.elapsed_ms == 500U);
    advance(-179.0f,3); CHECK(PID_GetStage() == PID_STAGE_ANGLE);
    command("AngleWait=0\n"); command("AngleLeft=10\n"); command("AngleRight=10\n");

    /* 运行中从各控制方式接管，只有 Track=1 重建本次起跑方向。 */
    fresh(35.0f); command("TurnTo=30\n"); PID_GetAngleStatus(&status);
    CHECK(PID_GetStage() == PID_STAGE_TURN && status.initial_deg == 179.0f && status.active_target_deg == -30.0f);
    elapsed = status.elapsed_ms;
    advance(35.0f,1); command("TurnTo=30\n"); PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 179.0f && status.elapsed_ms == elapsed + 20U);
    command("angle=1\n"); PID_GetAngleStatus(&status);
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN && status.active_target_deg == 70.0f && status.initial_deg == 179.0f);
    raw = 0x00; reads = ir_reads; advance(70.0f,3);
    CHECK(PID_GetStage() == PID_STAGE_STRAIGHT && ir_reads == reads);
    advance(70.0f,1); CHECK(left_pwm == 1420U && right_pwm == 1580U && ir_reads == reads);
    command("Stage=EDGE\n"); CHECK(PID_GetStage() == PID_STAGE_EDGE && left_pwm == 1500U && right_pwm == 1500U);
    command("angle=1\n"); CHECK(PID_GetStage() == PID_STAGE_STRAIGHT_ALIGN);
    command("Mode=0\n"); CHECK(PID_GetStage() == PID_STAGE_MANUAL && PID_IsRunning());
    command("Stage=ANGLE\n"); PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 179.0f && status.active_target_deg == 179.0f);
    fresh(-25.0f); command("Track=1\n"); PID_GetAngleStatus(&status);
    CHECK(PID_GetStage() == PID_STAGE_CENTER && status.initial_deg == -25.0f && status.elapsed_ms == 0U);
    command("Stage=STOP\n"); command("Mode=1\n"); CHECK(!PID_IsRunning() && PID_GetStage() == PID_STAGE_STOP);
    command("Stage=EDGE\n"); CHECK(PID_IsRunning() && PID_GetMode() == PID_MODE_RIGHT_EDGE);
    fresh(41.0f); command("Stage=ANGLE\n"); PID_GetAngleStatus(&status);
    CHECK(status.initial_deg == 41.0f && status.active_target_deg == 41.0f);

    /* 角度启动无有效IMU时必须停车；边缘循迹本身不依赖IMU。 */
    sample.healthy = 0U; command("Stage=CENTER\n");
    CHECK(strstr(output,"ERR") && PID_GetStage() == PID_STAGE_FAULT && !PID_IsRunning());
    command("Stage=EDGE\n"); CHECK(PID_IsRunning() && PID_GetStage() == PID_STAGE_EDGE);
    command("angle=1\n"); CHECK(strstr(output,"ERR") && !PID_IsRunning());
    fresh(0.0f); command("Stage=CENTER\n"); sample.fault_count++;
    command("TurnTo=0\n"); CHECK(strstr(output,"ERR") && PID_GetStage() == PID_STAGE_FAULT);
    fresh(0.0f); command("Stage=ANGLE\n"); tick += 101U;
    command("Stage=ANGLE\n"); CHECK(strstr(output,"ERR") && !PID_IsRunning());
    fresh(0.0f); command("Stage=CENTER\n"); tick += 101U;
    command("Track=1\n"); CHECK(strstr(output,"ERR") && !PID_IsRunning());

    /* 原地补偿叠加后仍受限幅，两个方向可通过蓝牙独立关闭。 */
    fresh(-11.0f); command("TurnFwdComp=80\n"); command("TurnRevComp=80\n"); command("TurnTo=0\n");
    PID_Update20ms(); CHECK(left_pwm == 1420U && right_pwm == 1420U);
    command("TurnFwdComp=0\n"); command("TurnRevComp=0\n");
    advance(-11.0f,1); CHECK(left_pwm == 1467U && right_pwm == 1467U);
    command("TurnMin=40\n"); command("TurnRevMin=40\n");
    command("Angle?\n"); CHECK(strstr(output,"FwdComp=40.000") && strstr(output,"RevComp=40.000"));
    command("TurnFwdComp=81\n"); CHECK(strstr(output,"ERR"));
    command("TurnRevComp=-1\n"); CHECK(strstr(output,"ERR"));
    command("TurnRevComp=inf\n"); CHECK(strstr(output,"ERR"));

    /* 三套增益按各自命令调整，中心与边缘仍独立。 */
    command("Mode=1\n"); command("Kp=35\n"); command("Ki=0.1\n"); command("Kd=1\n");
    command("Base=60\n"); command("LostExtra=20\n"); PID_GetParameters(&params);
    CHECK(params.mode == PID_MODE_RIGHT_EDGE && params.kp == 35.0f && params.base_speed_us == 60.0f);
    CHECK(params.ki > 0.09f && params.kd == 1.0f && params.lost_extra_us == 20.0f);
    command("PID?\n"); CHECK(strstr(output,"Mode=1") && strstr(output,"Kp=35.000"));
    command("Mode=0\n"); PID_GetParameters(&params); CHECK(params.kp == 40.0f && params.base_speed_us == 80.0f);
    command("AngleKp=4\n"); command("AngleKi=0.2\n"); command("AngleKd=1\n"); command("AngleBase=70\n");
    PID_GetAngleStatus(&status); CHECK(status.kp == 4.0f && status.base_us == 70.0f && status.turn_kp == 3.0f);
    command("Stage=STOP\n"); command("LeftSpeed=1460\n"); command("RightSpeed=1540\n");
    CHECK(left_pwm == 1460U && right_pwm == 1540U);
    command("Stage=STOP\n"); CHECK(left_pwm == 1500U && right_pwm == 1500U);

    command("IR=0\n"); output_size = 0; UART_IR(); drain(); CHECK(output_size == 0U);
    command("IR=1\n"); output_size = 0; raw = 0xE7; UART_IR(); drain();
    CHECK(strstr(output,"IR: 0xE7 11100111"));
    output_size = 0; UART_IR(); drain(); CHECK(output_size == 0U);
    raw = 0xFF; UART_IR(); drain(); CHECK(strstr(output,"IR: 0xFF 11111111"));

    /* 发送队列堵塞时 Stage=STOP 也必须先停车。 */
    fresh(0.0f); command("TurnTo=30\n"); PID_Update20ms(); CHECK(left_pwm != 1500U);
    memset(payload,'a',88); payload[88]='\n'; payload[89]=0;
    for (i=0;i<6;i++) UART_SendString(payload);
    feed("Stage=STOP\n"); CHECK(!PID_IsRunning() && left_pwm == 1500U && right_pwm == 1500U);
    drain(); UART_Task(); drain(); CHECK(strstr(output,"Stage=STOP OK"));
    /* 新增ACTIVE后，最大字段长度的整组回复仍放得进预留空间。 */
    tick = 100U; sample.fault_count = 0xFFFFFFFFU; sample.max_gap_ms = 0xFFFFFFFFU;
    fresh(179.0f); command("Track=1\n");
    command("AngleLeft=179.9\n"); command("AngleRight=179.9\n");
    command("AngleKp=1000\n"); command("AngleKi=1000\n"); command("AngleKd=1000\n");
    command("AngleBase=250\n"); command("AngleWait=2147483647\n");
    command("TurnMax=200\n"); command("TurnFwdComp=200\n"); command("TurnRevComp=200\n");
    command("TurnKp=1000\n"); command("TurnKi=1000\n"); command("TurnKd=1000\n");
    command("AngleTarget=179\n"); tick = 99U; fresh(-179.0f); command("Stage=ANGLE\n");
    memset(queued,'q',255); queued[255]=0;
    output_size = 0; UART_SendString(queued); feed("Angle?\n"); drain(); UART_Task(); drain();
    CHECK(strstr(output,"Elapsed=4294967295") && strstr(output,"Faults=4294967295"));
    CHECK(strstr(output,"Stage=AUTO_ALIGN") && strstr(output,"ACTIVE TargetYaw=179.000 Error=-2.000"));
    CHECK(strstr(output,"STRAIGHT Target=179.000 Error=0.000") && !strstr(output,"TX_OVERFLOW"));
    return 0;
}
