	#include "pid.h"
	#include "ir.h"
	#include "pwm.h"

	/*
	 * 第一版循迹参数，适用于固定 20ms 控制周期。
	 * 调参顺序：先只调 Kp，再增加 Kd，最后确有需要时再增加 Ki。
	 */
	volatile float pid_kp = 40.0f;
	volatile float pid_ki = 0.0f;
	volatile float pid_kd = 0.0f;
	volatile float pid_base_speed_us = 80.0f;

	#define PID_STOP_US          1500.0f
	#define PID_MAX_US       250.0f
	#define PID_REVERSE_DEADZONE_US   50.0f  /* 反转死区补偿具体多少比较好，还需测试，单位 us */
	#define PID_GAIN_MAX          1000.0f
	#define PID_I_ACTIVE_ERROR         1.0f
	#define PID_I_OUTPUT_LIMIT_US     20.0f

	static volatile uint8_t pid_running = 0;
	static float pid_error_sum = 0.0f;
	static float pid_last_error = 0.0f;
	static uint8_t pid_has_last_error = 0;

	static float PID_Limit(float value, float low, float high)
	{
		if (value > high)
		{
			return high;
		}
		if (value < low)
		{
			return low;
		}
		return value;
	}

	static void PID_ResetHistory(void)
	{
		pid_error_sum = 0.0f;
		pid_last_error = 0.0f;
		pid_has_last_error = 0;
	}

	static uint8_t PID_ValueInRange(float value, float high)
	{
		/* value != value 可以识别 NaN；正负无穷会被范围判断拒绝。 */
		if ((value != value) || (value < 0.0f) || (value > high))
		{
			return 0U;
		}
		return 1U;
	}

	static uint32_t PID_EnterCritical(void)
	{
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		return primask;
	}

	static void PID_ExitCritical(uint32_t primask)
	{
		if (primask == 0U)
		{
			__enable_irq();
		}
	}

	static void Car_stop(void)
	{
		PWM_LeftSpeed(1500U);
		PWM_RightSpeed(1500U);
	}

	static void PID_ApplyForward(float left_forward, float right_forward)
	{
		uint32_t left_pulse;
		uint32_t right_pulse;

		/* 负速度表示反转，先补偿实测死区，再限制最终输出幅度。 */
		if (left_forward < 0.0f)
		{
			left_forward -= PID_REVERSE_DEADZONE_US;
		}
		if (right_forward < 0.0f)
		{
			right_forward -= PID_REVERSE_DEADZONE_US;
		}

		left_forward = PID_Limit(left_forward,-PID_MAX_US,PID_MAX_US);//pid结果限制
		right_forward = PID_Limit(right_forward,-PID_MAX_US,PID_MAX_US);

		
		left_pulse = (uint32_t)(PID_STOP_US - left_forward + 0.5f);
		right_pulse = (uint32_t)(PID_STOP_US + right_forward + 0.5f);
		PWM_LeftSpeed(left_pulse);
		PWM_RightSpeed(right_pulse);
	}

	static float PID_CalculatePosition(uint8_t white_bits)
	{
		uint8_t i;
		uint8_t count = 0;
		uint16_t position_sum = 0;

		/* bit7 是最左，bit0 是最右；从左到右依次赋予 1~8 的权重。 */
		for (i = 0; i < 8; i++)
		{
			if ((white_bits & (uint8_t)(0x80U >> i)) != 0U)
			{
				position_sum = (uint16_t)(position_sum + i + 1U);
				count++;
			}
		}

		return (float)position_sum / (float)count;
	}

	void PID_Init(void)
	{
		pid_running = 0;
		PID_ResetHistory();
		Car_stop();
	}

	void PID_Start(void)
	{
		/* 最后才开放中断更新，避免复位状态时被 TIM3 使用到一半。 */
		pid_running = 0;
		PID_ResetHistory();
		Car_stop();
		pid_running = 1;
	}

	void PID_Stop(void)
	{
		/* 先禁止中断更新，再清历史和停车。 */
		pid_running = 0;
		PID_ResetHistory();
		Car_stop();
	}

	uint8_t PID_IsRunning(void)
	{
		return pid_running;
	}

	uint8_t PID_SetKp(float value)
	{
		uint32_t primask;

		if (PID_ValueInRange(value, PID_GAIN_MAX) == 0U)
		{
			return 0U;
		}
		primask = PID_EnterCritical();
		pid_kp = value;
		PID_ResetHistory();
		PID_ExitCritical(primask);
		return 1U;
	}

	uint8_t PID_SetKi(float value)
	{
		uint32_t primask;

		if (PID_ValueInRange(value, PID_GAIN_MAX) == 0U)
		{
			return 0U;
		}
		primask = PID_EnterCritical();
		pid_ki = value;
		PID_ResetHistory();
		PID_ExitCritical(primask);
		return 1U;
	}

	uint8_t PID_SetKd(float value)
	{
		uint32_t primask;

		if (PID_ValueInRange(value, PID_GAIN_MAX) == 0U)
		{
			return 0U;
		}
		primask = PID_EnterCritical();
		pid_kd = value;
		PID_ResetHistory();
		PID_ExitCritical(primask);
		return 1U;
	}

	uint8_t PID_SetBaseSpeed(float value)
	{
		uint32_t primask;

		if (PID_ValueInRange(value, PID_MAX_US) == 0U)
		{
			return 0U;
		}
		primask = PID_EnterCritical();
		pid_base_speed_us = value;
		PID_ExitCritical(primask);
		return 1U;
	}

	void PID_GetParameters(PID_Parameters *parameters)
	{
		uint32_t primask;

		if (parameters == NULL)
		{
			return;
		}
		primask = PID_EnterCritical();
		parameters->kp = pid_kp;
		parameters->ki = pid_ki;
		parameters->kd = pid_kd;
		parameters->base_speed_us = pid_base_speed_us;
		parameters->running = pid_running;
		PID_ExitCritical(primask);
	}

	void PID_Update20ms(void)
	{
		uint8_t raw_state;
		uint8_t white_bits;
		float position;
		float error;
		float derivative = 0.0f;
		float correction;
		float base_speed;
		float kp;
		float ki;
		float kd;

		if (pid_running == 0U)
		{
			return;
		}

		raw_state = IR_Read();
		white_bits = (uint8_t)(~raw_state);  /* 原始 0 表示白线，转换后 1 表示白线 */
		kp = pid_kp;
		ki = pid_ki;
		kd = pid_kd;
		base_speed = pid_base_speed_us;

		if (white_bits == 0x00U)
		{
			/* 全部位于黑底：清历史，保持上一拍电机输出。 */
			PID_ResetHistory();
			return;
		}

		position = PID_CalculatePosition(white_bits);
		error = 4.5f - position;

		if ((ki > 0.0f) &&(error >= -PID_I_ACTIVE_ERROR) &&(error <= PID_I_ACTIVE_ERROR))//积分死去
		{
			float error_sum_limit = PID_I_OUTPUT_LIMIT_US / ki;
			pid_error_sum += error;
			pid_error_sum = PID_Limit(pid_error_sum, -error_sum_limit,error_sum_limit);
		}
		else
		{
			pid_error_sum = 0.0f;
		}

		/* 中间两路同时见到白线时，主动消除残留积分。 */
		if ((white_bits & 0x18U) == 0x18U)
		{
			pid_error_sum = 0.0f;
		}

		if (pid_has_last_error != 0U)
		{
			derivative = error - pid_last_error;
		}
		pid_last_error = error;
		pid_has_last_error = 1;

		correction = kp * error +
					 ki * pid_error_sum +
					 kd * derivative;

		/* correction > 0 时左侧减速、右侧加速，较大时允许内侧轮反转。 */
		PID_ApplyForward(base_speed - correction,
						 base_speed + correction);
	}
