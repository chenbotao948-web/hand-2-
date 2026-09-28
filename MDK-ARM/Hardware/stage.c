#include "imu.h"
#include "pwm.h"
#include "ir.h"
#include "stage.h"
#include "pid.h"
#include "oled.h"


static float angle_left_deg = 10.0f, angle_right_deg = 10.0f;//容忍角度误差
static float first_yaw;//出发时的角度
static Stage_State current_stage;
uint32_t tim3_pid_tick_count = 0;  /* TIM3 20ms 中断计数，便于调试观察 */
uint16_t angle_wait=1000;//等待的时间
float now_yaw;//当前角度
uint8_t white_bits;//当前灰度

void  Stage_Init(void)
{
	current_stage=STAGE_STOP;
	IMU_GetAngle(&first_yaw);
	first_yaw=IMU_WrapAngle(first_yaw);
}
void  Stage_Update(void)
{
	float delta_yaw;

	IMU_GetAngle(&now_yaw);
	now_yaw=IMU_WrapAngle(now_yaw);
	OLED_ShowSignedNum(1,1,now_yaw,4);
	OLED_ShowSignedNum(2,1,12,4);
	delta_yaw = IMU_WrapAngle(now_yaw - first_yaw);
	if(current_stage==STAGE_GO)//一开始没有线，所以要先出发一会儿，等到找到线了，我们就切换下一个状态。
    {
    PWM_LeftSpeed(1420);
	PWM_RightSpeed(1580);
		if(!IR_Read())
		{
			current_stage=STAGE_CENTER;
		}
    }

	if(current_stage==STAGE_CENTER &&  tim3_pid_tick_count>angle_wait && delta_yaw >= -angle_right_deg && delta_yaw <= angle_left_deg && IR_Read() == 0xFF)
    {
		current_stage=STAGE_ANGLE;
		angle_turn_done = 0U;
		angle_target_yaw=first_yaw;
		PID_ResetTurnHistory();
    }
	
	if(current_stage==STAGE_CENTER)//中心状态
    {
	    white_bits = (~IR_Read());
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