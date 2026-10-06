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
static volatile uint8_t catch_flag;
static volatile uint8_t catch_reacquiring_line;
static volatile uint32_t catch_start_tick;//抓取开始那一刻的时间，不用改，主要后面两个
static volatile uint32_t catch_entry_ms = STAGE_DEFAULT_CATCH_TIME_MS;//什么时候开始抓取,这里单位是ms要注意
static volatile uint16_t catch_duration_seconds = STAGE_DEFAULT_CATCH_DURATION_SECONDS;//抓取持续时间

static volatile uint8_t time_gate_unlocked;
static volatile uint16_t angle_wait_seconds = STAGE_DEFAULT_WAIT_SECONDS;
/* SpeedTime 独立于 Time=，单位为秒，从 run 起算 CENTER 减速时刻。 */
static volatile uint16_t center_slowdown_seconds = STAGE_DEFAULT_SPEED_TIME_SECONDS;

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
	center_slowdown_seconds = STAGE_DEFAULT_SPEED_TIME_SECONDS;
	catch_entry_ms = STAGE_DEFAULT_CATCH_TIME_MS;
	catch_duration_seconds = STAGE_DEFAULT_CATCH_DURATION_SECONDS;
	catch_flag = 0U;
	catch_reacquiring_line = 0U;
	catch_start_tick = 0U;
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
	if ((catch_reacquiring_line != 0U) && (white_bits != 0U))
	{
		catch_reacquiring_line = 0U;
	}
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

	/* TIM3 每 20 ms 检查：本次 run 仅在 CENTER 按起跑时间触发一次拾取。 */
	if ((current_stage == STAGE_CENTER) && (run_timer_active != 0U) &&
	    (catch_flag == 0U) &&
	    ((uint32_t)(HAL_GetTick() - run_start_tick) >=
	     catch_entry_ms))//什么时候开始抓取
	{
		catch_flag = 1U;
		Stage_SetState(STAGE_CATCH);
	}

	if((current_stage==STAGE_CENTER) && (run_timer_active != 0U) &&
	   (time_gate_unlocked != 0U) &&
	   (catch_reacquiring_line == 0U) &&
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
		/* 从 run 起算，达到 SpeedTime 后基础速度由 80 降为 50；Time= 只控制进入 ANGLE。 */
        if ((run_timer_active != 0U) &&
            ((uint32_t)(HAL_GetTick() - run_start_tick) >=
             (uint32_t)center_slowdown_seconds * 1000U))
        {
            (void)PID_SetCenterBaseSpeed(50.0f);//什么时候减速
        }
        else
        {
            (void)PID_SetCenterBaseSpeed(80.0f);
        }
        PID_UpdateCenter(white_bits);
		return;
    }
	
		if(current_stage==STAGE_CATCH)//拾取状态
    {
	/*
	
	拾取放在这里
	
	
	
	*/
		if ((uint32_t)(HAL_GetTick() - catch_start_tick) >=
		    (uint32_t)catch_duration_seconds * 1000U)//时间比较
		{
			Stage_SetState(STAGE_CENTER);
			catch_reacquiring_line = (raw_ir == 0xFFU) ? 1U : 0U;//成立取1，不成立取0
			if ((run_timer_active != 0U) &&
			    ((uint32_t)(HAL_GetTick() - run_start_tick) >=
			     (uint32_t)center_slowdown_seconds * 1000U))
			{
				(void)PID_SetCenterBaseSpeed(50.0f);
			}
			else
			{
				(void)PID_SetCenterBaseSpeed(80.0f);
			}
			PID_UpdateCenter((catch_reacquiring_line != 0U) ? 0x18U : white_bits);//跟上面联动，如果进入小黑圈的话，没看到线先直走
		}
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
	catch_reacquiring_line = 0U;
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


        case STAGE_CATCH:
        {
            /* 进入拾取时停车，并记录停留起点（毫秒）。 */
            catch_start_tick = HAL_GetTick();
            PWM_LeftSpeed(1500U);
            PWM_RightSpeed(1500U);
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
	catch_reacquiring_line = 0U;
	run_start_tick = HAL_GetTick();
	run_timer_active = 1U;
	time_gate_unlocked = (angle_wait_seconds == 0U) ? 1U : 0U;
	catch_flag = 0U;
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
	catch_reacquiring_line = 0U;
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
/* 串口修改独立减速门槛时，只更新秒数，保留本次 run 的已计时间。 */
uint8_t Stage_SetSpeedTimeSeconds(uint16_t seconds)
{
    uint32_t key;
    if (seconds > STAGE_MAX_WAIT_SECONDS) return 0U;
    key = Stage_Lock();
    center_slowdown_seconds = seconds;
    Stage_Unlock(key);
    return 1U;
}

/* 串口运行中修改秒数时保留 run 或 CATCH 已计时间。 */
uint8_t Stage_SetCatchTimeMilliseconds(uint32_t milliseconds)
{
    uint32_t key;
    if (milliseconds > (uint32_t)STAGE_MAX_WAIT_SECONDS * 1000U) return 0U;
    key = Stage_Lock();
    catch_entry_ms = milliseconds;
    Stage_Unlock(key);
    return 1U;
}

uint8_t Stage_SetCatchTimeSeconds(uint16_t seconds)
{
    return Stage_SetCatchTimeMilliseconds((uint32_t)seconds * 1000U);
}

uint8_t Stage_SetCatchDurationSeconds(uint16_t seconds)
{
    uint32_t key;
    if ((seconds == 0U) || (seconds > STAGE_MAX_WAIT_SECONDS)) return 0U;
    key = Stage_Lock();
    catch_duration_seconds = seconds;
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