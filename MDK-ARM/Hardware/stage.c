#include "imu.h"
#include "pwm.h"
#include "ir.h"
#include "stage.h"
#include "pid.h"


static float angle_left_deg = 45.0f, angle_right_deg = 45.0f;//容忍角度误差
static float first_yaw;//出发时的角度
static Stage_State current_stage;
uint32_t tim3_pid_tick_count = 0;  /* TIM3 20ms 中断计数，便于调试观察 */
float now_yaw;//当前角度
uint8_t white_bits;//当前灰度

static volatile uint32_t run_start_tick;
static volatile uint8_t run_timer_active;
static volatile uint8_t time_gate_unlocked;
static volatile uint16_t angle_wait_seconds = STAGE_DEFAULT_WAIT_SECONDS;

static uint32_t Stage_Lock(void)
{
	uint32_t key = __get_PRIMASK();
	__disable_irq();
	return key;
}

static void Stage_Unlock(uint32_t key)
{
	if (key == 0U) __enable_irq();
}

void  Stage_Init(void)
{
	current_stage=STAGE_CENTER;
	run_start_tick = HAL_GetTick();
	run_timer_active = 0U;
	time_gate_unlocked = 0U;
	angle_wait_seconds = STAGE_DEFAULT_WAIT_SECONDS;
	IMU_GetAngle(&first_yaw);
	first_yaw=IMU_WrapAngle(first_yaw);
}
void  Stage_Update(void)
{
	float delta_yaw;
	uint8_t raw_ir;

	/* IR_Read: black=1, white=0. Use one sample for this control tick. */
	raw_ir = IR_Read();
	white_bits = (uint8_t)~raw_ir;
	//这里搞了一个标志位
	if ((run_timer_active != 0U) && (time_gate_unlocked == 0U) &&
	    ((uint32_t)(HAL_GetTick() - run_start_tick) >=
	     (uint32_t)angle_wait_seconds * 1000U))
	{
		time_gate_unlocked = 1U;
	}
	IMU_GetAngle(&now_yaw);
	now_yaw=IMU_WrapAngle(now_yaw);
	delta_yaw = IMU_WrapAngle(now_yaw - first_yaw);
	if(current_stage==STAGE_GO)//一开始没有线，所以要先出发一会儿，等到找到线了，我们就切换下一个状态。
    {
    PWM_LeftSpeed(1420);
	PWM_RightSpeed(1580);
		if(raw_ir != 0xFF)
		{
			current_stage=STAGE_CENTER;
		}
    }

	if((current_stage==STAGE_CENTER) && (run_timer_active != 0U) &&
	   (time_gate_unlocked != 0U) &&
	   (delta_yaw >= -angle_right_deg) && (delta_yaw <= angle_left_deg) &&
	   (raw_ir == 0xFFU))//如果上一个状态是中心检测角度在误差的容忍状态之内，识别到的的线就是全黑，时间也允许
    {
		current_stage=STAGE_ANGLE;
		angle_turn_done = 0U;
		angle_target_yaw=first_yaw;
		PID_ResetTurnHistory();
    }
	
	if(current_stage==STAGE_CENTER)//中心状态
    {
		PID_UpdateCenter(white_bits);
		return;
    }
	
	
	if(current_stage==STAGE_ANGLE)//角度状态
    {
        PID_UpdateAngle(now_yaw);
		if(white_bits!=0U)
        {
		Stage_SetState(STAGE_EDGE);
        }
    }
	
	if(current_stage==STAGE_EDGE)
    {
		PID_UpdateRightEdge(white_bits);
		return;
    }
	
	if(current_stage==STAGE_STOP)
    {
		
		return;
    }

}
/*通过函数来修改别的文件的参数，而不使用全局变量。*/
void Stage_SetState(Stage_State state)
{
    /* 已经是这个状态，就不用重复初始化 */
    if (state == current_stage)
    {
        return;
    }

    /* 先切换状态 */
    current_stage = state;


    /* 根据新状态做进入初始化 */
    switch (current_stage)
    {
        case STAGE_GO:
        {
            /* Go 暂时没有特殊初始化 */
            break;
        }


        case STAGE_CENTER:
        {
            /* Center 暂时没有特殊初始化 */
            break;
        }


        case STAGE_ANGLE:
        {
            /*
             * 每次重新进入 Angle，
             * 都认为原地转向还没完成
             */
            angle_turn_done = 0U;

            /* 原地转向目标：回到起始方向 */
            angle_target_yaw = first_yaw;

            /* 清掉上一次原地转向 PID 历史 */
            PID_ResetTurnHistory();

            break;
        }


        case STAGE_EDGE:
        {
            /* 进入边沿循迹时清边沿历史 */
            PID_ResetEdgeHistory();

            break;
        }


        case STAGE_STOP:
        {
            /* 立即停车 */
            PWM_LeftSpeed(1500U);
            PWM_RightSpeed(1500U);

            break;
        }
    }
}

Stage_State Stage_GetState(void)
{
    return current_stage;
}
/**
 * @brief（简介）发送串口run进行一系列操作。
 * @param  （参数）
 * @retval （返回值）
 * @note  （说明）
 */
void Stage_StartRun(void)
{
	uint32_t key = Stage_Lock();
	run_start_tick = HAL_GetTick();
	run_timer_active = 1U;
	time_gate_unlocked = (angle_wait_seconds == 0U) ? 1U : 0U;
	Stage_SetState(STAGE_GO);
	Stage_Unlock(key);
}
/**//**
 * @brief（简介）时间停止
 * @param  （参数）
 * @retval （返回值）
 * @note  （说明）换状态的时候会调用一次
 */
void Stage_ResetRunTimer(void)
{
	uint32_t key = Stage_Lock();
	run_timer_active = 0U;
	time_gate_unlocked = 0U;
	Stage_Unlock(key);
}

uint8_t Stage_SetWaitSeconds(uint16_t seconds)
{
	uint32_t key;
	uint32_t elapsed_ms;
	if (seconds > STAGE_MAX_WAIT_SECONDS) return 0U;
	key = Stage_Lock();
	angle_wait_seconds = seconds;
	if (run_timer_active != 0U)
	{
		elapsed_ms = (uint32_t)(HAL_GetTick() - run_start_tick);
		time_gate_unlocked = (elapsed_ms >= (uint32_t)seconds * 1000U) ? 1U : 0U;
	}
	else
	{
		time_gate_unlocked = 0U;
	}
	Stage_Unlock(key);
	return 1U;
}
/**
 * @brief（简介）已经过了几秒
 * @param  （参数）
 * @retval （返回值）经过的时间
 * @note  （说明）它的这个词是串口发送忘之后开始计时，开始算时间。
 */
uint32_t Stage_GetElapsedSeconds(void)
{
	uint32_t key = Stage_Lock();
	uint32_t elapsed_ms = 0U;
	if (run_timer_active != 0U)
		elapsed_ms = (uint32_t)(HAL_GetTick() - run_start_tick);
	Stage_Unlock(key);
	return elapsed_ms / 1000U;
}