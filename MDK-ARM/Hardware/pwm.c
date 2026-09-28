#include "pwm.h"

extern TIM_HandleTypeDef htim3;

void PWM_Start(void)
{
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**
  * @brief  设置左侧电机速度
  * @param  speed: Tsig 值，即单脉冲信号低电平时间，单位 us。
  *                实测左侧小于 1500us 前进，右侧大于 1500us 前进；
  *                两侧 1500us 均为停车点。
  *                脉冲周期固定为 20ms。
  * @note   PWM_LeftSpeed 设置 CH1/CH3，PWM_RightSpeed 设置 CH2/CH4。
  */
void PWM_LeftSpeed(uint32_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
}

void PWM_RightSpeed(uint32_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
}

void PWM_Init(void)
{
	PWM_LeftSpeed(1500);
	PWM_RightSpeed(1500);
	PWM_Start();
}

void Car_Stop(void)
{
	PWM_LeftSpeed(1500);
	PWM_RightSpeed(1500);
}
