#include <string.h>
#include "usart.h"
#include "pwm.h"
#include <stdlib.h>
#include "pid.h"
#include "stage.h"
static uint8_t UART_ParseFloat(const char *text, float *value);
static uint8_t UART_ParseTimeSeconds(const char *text, uint16_t *seconds);
static uint8_t UART_ParseCatchTimeMilliseconds(const char *text, uint32_t *milliseconds);
static void UART_SendString(const char *str);
static void UART_SendResult(const char *line, uint8_t success);
/*函数提前定义*/
extern UART_HandleTypeDef huart1;

#define UART_RX_BUFFER_SIZE 64

static uint8_t rx_byte;

/* 用来保存一整行 */
static char rx_buffer[UART_RX_BUFFER_SIZE];

/* 当前写到数组第几个位置 */
static uint16_t rx_index = 0U;

/* 0=还没收完整，1=已经收到完整一行 */
static volatile uint8_t rx_line_ready = 0U;
extern UART_HandleTypeDef huart1;

static uint8_t rx_byte;

void UART_Init(void)
{
    rx_index = 0U;
    rx_line_ready = 0U;

    HAL_UART_Receive_IT(&huart1,
                        &rx_byte,
                        1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 上一行还没处理，就先不往数组里写 */
        if (rx_line_ready == 0U)
        {
            /* 收到换行，说明一整行结束 */
            if (rx_byte == '\n')
            {
                if (rx_index > 0U)
                {
                    rx_buffer[rx_index] = '\0';

                    rx_line_ready = 1U;//命令结束标志位
                }
            }

            /* '\r' 直接忽略 */
            else if (rx_byte != '\r')
            {
                if (rx_index < UART_RX_BUFFER_SIZE - 1U)//判断有没有溢出
                {
                    rx_buffer[rx_index] = (char)rx_byte;

                    rx_index++;
                }
            }
        }

        /* 继续接下一个字节 */
        HAL_UART_Receive_IT(&huart1,
                            &rx_byte,
                            1U);
    }
}

//命令对比
static void UART_ProcessLine(char *line)
{
    float value;
    uint8_t success;
    uint16_t seconds;
    uint32_t catch_time_ms;
    if (strcmp(line, "run") == 0)
    {
        Stage_StartRun();
        UART_SendString("RUN OK\r\n");
    }
    else if (strcmp(line, "STOP") == 0)
    {
		Stage_ResetRunTimer();
		Stage_SetState(STAGE_STOP);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STOP OK\r\n",
                          9U,
                          100U);
    }

    else if (strcmp(line, "GO") == 0)
    {
		Stage_ResetRunTimer();
		Stage_SetState(STAGE_GO);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"GO OK\r\n",
                          7U,
                          100U);
    }
	    else if (strcmp(line, "CENTER") == 0)
    {
		Stage_ResetRunTimer();
		Stage_SetState(STAGE_CENTER);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"CENTER OK\r\n",
                          11U,
                          100U);
    }

	/* SpeedTime= 调整 run 后 CENTER 的减速秒数；Time= 仍控制阶段切换。 */
	else if (strncmp(line, "SpeedTime=", 10) == 0)
	{
    success = UART_ParseTimeSeconds(line + 10, &seconds);
    if (success != 0U) success = Stage_SetSpeedTimeSeconds(seconds);
    UART_SendResult(line, success);
	}
	else if (strncmp(line, "CatchTime=", 10) == 0)
	{
		success = UART_ParseCatchTimeMilliseconds(line + 10, &catch_time_ms);
		if (success != 0U) success = Stage_SetCatchTimeMilliseconds(catch_time_ms);
		UART_SendResult(line, success);
	}
	else if (strncmp(line, "CatchDuration=", 14) == 0)
	{
		success = UART_ParseTimeSeconds(line + 14, &seconds);
		if (success != 0U) success = Stage_SetCatchDurationSeconds(seconds);
		UART_SendResult(line, success);
	}
	else if (strncmp(line, "Time=", 5) == 0)
	{
		success = UART_ParseTimeSeconds(line + 5, &seconds);
		if (success != 0U) success = Stage_SetWaitSeconds(seconds);
		UART_SendResult(line, success);
	}
	else if (strncmp(line, "Kp=", 3) == 0)
	{
		success = UART_ParseFloat(line + 3, &value);
	
		if (success != 0U)
		{
			success = PID_SetKp(value);
		}
	
		UART_SendResult(line, success);
	}


	else if (strncmp(line, "Ki=", 3) == 0)
	{
		success = UART_ParseFloat(line + 3, &value);//注意这里的返回值
	
		if (success != 0U)
		{
			success = PID_SetKi(value);
		}
	
		UART_SendResult(line, success);
	}


		else if (strncmp(line, "Kd=", 3) == 0)
	{
		success = UART_ParseFloat(line + 3, &value);
	
		if (success != 0U)
		{
			success = PID_SetKd(value);
		}
	
		UART_SendResult(line, success);
	}
	
		else if (strncmp(line, "Ekp=", 4) == 0)
	{
		success = UART_ParseFloat(line + 4, &value);
	
		if (success != 0U)
		{
			success = PID_SetEKp(value);
		}
	
		UART_SendResult(line, success);
	}


	else if (strncmp(line, "Eki=", 4) == 0)
	{
		success = UART_ParseFloat(line + 4, &value);//注意这里的返回值
	
		if (success != 0U)
		{
			success = PID_SetEKi(value);
		}
	
		UART_SendResult(line, success);
	}


		else if (strncmp(line, "Ekd=", 4) == 0)
	{
		success = UART_ParseFloat(line + 4, &value);
	
		if (success != 0U)
		{
			success = PID_SetEKd(value);
		}
	
		UART_SendResult(line, success);
	}
		/* Base= 在 CENTER 中会被定时速度 80/50 于下一个控制周期覆盖。 */
		else if (strncmp(line, "CenterBase=", 11) == 0)
	{
		success = UART_ParseFloat(line + 11, &value);
	
		if (success != 0U)
		{
			success = PID_SetCenterBaseSpeed(value);
		}
	
		UART_SendResult(line, success);
	}
	
		else if (strncmp(line, "EdgeBase=", 9) == 0)
	{
		success = UART_ParseFloat(line + 9, &value);
	
		if (success != 0U)
		{
			success = PID_SetEdgeBaseSpeed(value);
		}
	
		UART_SendResult(line, success);
	}	
	else if (strcmp(line, "Kp?") == 0)
{
    float kp;
    char text[32];

    kp = PID_GetKp();

    sprintf(text, "Kp=%.2f\r\n", kp);//把格式化的内容“写进一个字符串数组

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)text,
                      strlen(text),
                      100U);
}
else if (strcmp(line, "PID?") == 0)
{
    PID_Parameters parameters;

    uint32_t kp_milli;
    uint32_t ki_milli;
    uint32_t kd_milli;
    uint32_t base_milli;

    char text[128];

    PID_GetParameters(&parameters);

    kp_milli =
        (uint32_t)(parameters.kp * 1000.0f + 0.5f);//用“乘 1000 → 整数拆小数”的办法来避免浮点 printf


    ki_milli =
        (uint32_t)(parameters.ki * 1000.0f + 0.5f);

    kd_milli =
        (uint32_t)(parameters.kd * 1000.0f + 0.5f);

    base_milli =
        (uint32_t)(parameters.base_speed_us * 1000.0f + 0.5f);


    sprintf(text,
            "PID Kp=%lu.%03lu Ki=%lu.%03lu Kd=%lu.%03lu Base=%lu.%03lu\r\n",

            (unsigned long)(kp_milli / 1000U),
            (unsigned long)(kp_milli % 1000U),

            (unsigned long)(ki_milli / 1000U),
            (unsigned long)(ki_milli % 1000U),

            (unsigned long)(kd_milli / 1000U),
            (unsigned long)(kd_milli % 1000U),

            (unsigned long)(base_milli / 1000U),
            (unsigned long)(base_milli % 1000U));


    HAL_UART_Transmit(&huart1,
                      (uint8_t *)text,
                      strlen(text),
                      100U);
}
					/*查照阶段*/
	else if (strcmp(line, "STATE?") == 0)
{
    Stage_State state;

    state = Stage_GetState();

     if (state == STAGE_GO)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STATE=GO\r\n",
                          10U,
                          100U);
    }
    else if (state == STAGE_CATCH)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STATE=CATCH\r\n",
                          13U,
                          100U);
    }
    else if (state == STAGE_CENTER)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STATE=CENTER\r\n",
                          14U,
                          100U);
    }
    else if (state == STAGE_ANGLE)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STATE=ANGLE\r\n",
                          13U,
                          100U);
    }
    else if (state == STAGE_EDGE)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STATE=EDGE\r\n",
                          12U,
                          100U);
    }
    else if (state == STAGE_STOP)
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STATE=STOP\r\n",
                          12U,
                          100U);
    }
}
/*设置状态*/
	else if (strncmp(line, "Stage=", 6) == 0)
{
    if (strcmp(line + 6, "CENTER") == 0)
    {
        Stage_ResetRunTimer();
        Stage_SetState(STAGE_CENTER);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STAGE CENTER OK\r\n",
                          17U,
                          100U);
    }

    else if (strcmp(line + 6, "ANGLE") == 0)
    {
        Stage_ResetRunTimer();
        Stage_SetState(STAGE_ANGLE);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STAGE ANGLE OK\r\n",
                          16U,
                          100U);
    }

    else if (strcmp(line + 6, "EDGE") == 0)
    {
        Stage_ResetRunTimer();
        Stage_SetState(STAGE_EDGE);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STAGE EDGE OK\r\n",
                          15U,
                          100U);
    }

    else if (strcmp(line + 6, "STOP") == 0)
    {
        Stage_ResetRunTimer();
        Stage_SetState(STAGE_STOP);

        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STAGE STOP OK\r\n",
                          15U,
                          100U);
    }

    else
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"STAGE ERR\r\n",
                          11U,
                          100U);
    }
}
    else
    {
        HAL_UART_Transmit(&huart1,
                          (uint8_t *)"ERR\r\n",
                          5U,
                          100U);
    }
	
}

void UART_Task(void)
{
    if (rx_line_ready == 0U)
    {
        return;
    }


    UART_ProcessLine(rx_buffer);

    rx_index = 0U;
    rx_line_ready = 0U;
}
/**
 * @brief（简介）解析数字,value这个的值最后会变成test的数字值
 * @param  （参数）要填入的参数
 * @param  （参数）想要赋值的参数
 * @retval （返回值）1成功
 * @note  （说明）strtof(text, &end)这个语法不会可以去问ai,调用了,主要是看数字后面的结尾是什么。比如123abc，返回abc
 */
static uint8_t UART_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return 0U;
    }

    parsed = strtof(text, &end);//数字解析命令有一个专门的调用

    if ((end == text) ||
        (*end != '\0') ||
        (parsed != parsed))
    {
        return 0U;
    }

    *value = parsed;

    return 1U;
}

/* CatchTime= 接受整数或一到两位小数，按 10 ms 整数单位解析，避免浮点误差。 */
static uint8_t UART_ParseCatchTimeMilliseconds(const char *text, uint32_t *milliseconds)
{
    uint32_t whole = 0U;
    uint32_t hundredths = 0U;
    uint8_t decimals = 0U;
    uint8_t digit;

    if ((text == NULL) || (milliseconds == NULL) || (*text < '0') || (*text > '9')) return 0U;
    while ((*text >= '0') && (*text <= '9'))
    {
        digit = (uint8_t)(*text - '0');
        if (whole > (STAGE_MAX_WAIT_SECONDS - digit) / 10U) return 0U;
        whole = whole * 10U + digit;
        text++;
    }
    if (*text == '.')
    {
        text++;
        while ((*text >= '0') && (*text <= '9') && (decimals < 2U))
        {
            hundredths = hundredths * 10U + (uint8_t)(*text - '0');
            decimals++;
            text++;
        }
        if (decimals == 0U) return 0U;
        if (decimals == 1U) hundredths *= 10U;
    }
    if ((*text != '\0') || ((whole == STAGE_MAX_WAIT_SECONDS) && (hundredths != 0U))) return 0U;
    *milliseconds = whole * 1000U + hundredths * 10U;
    return 1U;
}

static uint8_t UART_ParseTimeSeconds(const char *text, uint16_t *seconds)
{
    uint32_t parsed = 0U;
    uint8_t digit;

    if ((text == NULL) || (seconds == NULL) || (*text == '\0')) return 0U;
    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9')) return 0U;
        digit = (uint8_t)(*text - '0');
        if (parsed > (STAGE_MAX_WAIT_SECONDS - digit) / 10U) return 0U;
        parsed = parsed * 10U + digit;
        text++;
    }
    *seconds = (uint16_t)parsed;
    return 1U;
}
			/*发送一串字符*/
static void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1,
                      (uint8_t *)str,
                      strlen(str),
                      100U);
}

static void UART_SendResult(const char *line, uint8_t success)
{
    UART_SendString(line);

    if (success != 0U)
    {
        UART_SendString(" OK\r\n");
    }
    else
    {
        UART_SendString(" ERR\r\n");
    }
}