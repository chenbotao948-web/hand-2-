/* usart.c */
#include "usart.h"
#include "pwm.h"
#include "ir.h"
#include "pid.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* huart1 在 main.c 中定义，这里用 extern 引用 */
extern UART_HandleTypeDef huart1;

/* 接收缓冲区大小，命令最长 63 字节（留 1 字节放 '\0'） */
#define UART_RX_BUFFER_SIZE 64

static uint8_t rx_byte;                        /* 当前收到的 1 字节 */
static char rx_buffer[UART_RX_BUFFER_SIZE];    /* 一行命令的缓存 */
static uint16_t rx_index = 0;                  /* rx_buffer 当前写入位置，数组放的位置 */
static volatile uint8_t rx_line_ready = 0;     /* 1=完整命令，2=超长命令错误，命令标志位 */
static volatile uint8_t rx_drop_line = 0;      /* 丢弃当前残缺行，直到收到 '\n' */
static volatile uint8_t rx_drop_report = 0;    /* 当前丢弃行为超长命令，需要报错 */
static uint8_t ir_report_enabled = 1;           /* 默认输出红外状态变化 */

static void UART_StartReceive(void);
static void UART_ProcessLine(char *line);//程序对比表格
static uint8_t UART_ParseFloat(const char *text, float *value);
static void UART_SendCommandResult(const char *line, uint8_t success);
static void UART_SendPidParameters(void);

/**
  * @brief  串口初始化，启动第一次中断接收
  * @note   在 main 的 USER CODE BEGIN 2 中调用，
  *         必须在 MX_USART1_UART_Init() 之后。
  */
void UART_Init(void)
{
    rx_index = 0;//数组位置为零
    rx_line_ready = 0;
    rx_drop_line = 0;
    rx_drop_report = 0;
    ir_report_enabled = 1;
    UART_StartReceive();//接收中断开启
}

/**
  * @brief  启动一次中断接收，接收 1 字节到 rx_byte
  * @note   每处理完 1 字节都要重新调用一次，否则只收一次就停了。
  */
static void UART_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/**
  * @brief  阻塞方式发送一个字符串
  * @param  str: 以 '\0' 结尾的字符串
  * @note   用 HAL_MAX_DELAY 阻塞等待，简单场景够用。
  */
void UART_SendString(const char *str)
{
    if (str == NULL) return;
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief  在主循环中处理已经接收完成的命令
  * @note   命令解析、PWM 设置和阻塞回显都在这里执行，不占用串口中断。
  *         处理期间 rx_line_ready 保持为 1，中断不会覆盖 rx_buffer。
  */
void UART_Task(void)
{
    if (rx_line_ready == 0)
    {
        return;
    }

    if (rx_line_ready == 2U)
    {
        UART_SendString("ERR\n");
    }
    else
    {
        UART_ProcessLine(rx_buffer);
    }
    rx_line_ready = 0;
}

/**
  * @brief  串口接收完成回调，每收到 1 字节进一次
  * @param  huart: 触发回调的串口句柄
  * @note   - 上一条命令还没被主循环处理完时，新行整行丢弃；
  *         - 收到 '\n' 表示一行结束，通知主循环处理；
  *         - '\r' 直接忽略，兼容 "\r\n" 结尾。
  *         适合手动逐条发送，并在收到回复后再发下一条。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_drop_line != 0U)
        {
            if (rx_byte == '\n')
            {
                rx_drop_line = 0;
                rx_index = 0;
                if (rx_drop_report != 0U)
                {
                    rx_line_ready = 2U;
                    rx_drop_report = 0;
                }
            }
            UART_StartReceive();
            return;
        }

        /* 上一条还没处理完时，若新行已经开始，则整行丢弃。 */
        if (rx_line_ready != 0)
        {
            if (rx_byte != '\n' && rx_byte != '\r')
            {
                rx_drop_line = 1;
                rx_drop_report = 0;
            }
            UART_StartReceive();
            return;
        }

        if (rx_byte == '\n')//遇到串口通信结尾
        {
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';
                rx_line_ready = 1;
                rx_index = 0;
            }
        }
        else if (rx_byte != '\r')
        {
            if (rx_index < (UART_RX_BUFFER_SIZE - 1))//0到62存字符rx_index < 63。结尾还要放\0作为字符串结尾
            {
                rx_buffer[rx_index++] = rx_byte;
            }
            else
            {
                /* 超长命令不能截断执行，丢弃到行尾后在主循环报错。 */
                rx_index = 0;
                rx_drop_line = 1;
                rx_drop_report = 1;
            }
        }

        /* 无论哪种情况，都要重新开启下一次接收 */
        UART_StartReceive();
    }
}

/**
  * @brief  串口接收错误恢复
  * @note   ORE 会终止 HAL 的中断接收。清空当前行并重新启动接收，
  *         收到下一个 '\n' 后恢复正常命令边界；中断内不发送错误信息。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart->Instance == USART1) &&
	((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0))
    {
        rx_index = 0;
        rx_drop_line = 1;
        rx_drop_report = 0;
        UART_StartReceive();
    }
}

/**
  * @brief  解析一行命令并执行
  * @param  line: 以 '\0' 结尾的一行命令字符串
  * @note   支持电机、循迹、红外回报和 PID 实时调参命令。
  *         成功执行后回显 "原命令 OK\n"；否则回显 "原命令 ERR\n"。
  */
static void UART_ProcessLine(char *line)
{
    uint32_t speed;
    float value;
    uint8_t success = 1U;

    if (strcmp(line, "Track=1") == 0)
    {
        PID_Start();
    }
    else if (strcmp(line, "Track=0") == 0)
    {
        PID_Stop();
    }
    else if (strcmp(line, "Mode=0") == 0)
    {
        success = PID_SetMode(PID_MODE_CENTER);
    }
    else if (strcmp(line, "Mode=1") == 0)
    {
        success = PID_SetMode(PID_MODE_RIGHT_EDGE);
    }
    else if (strcmp(line, "IR=1") == 0)
    {
        ir_report_enabled = 1U;
    }
    else if (strcmp(line, "IR=0") == 0)
    {
        ir_report_enabled = 0U;
    }
    else if (strcmp(line, "PID?") == 0)
    {
        UART_SendPidParameters();
        return;
    }
    else if (strncmp(line, "Kp=", 3) == 0)
    {
        success = UART_ParseFloat(line + 3, &value);
        if (success != 0U)
        {
            success = PID_SetKp(value);
        }
    }
    else if (strncmp(line, "Ki=", 3) == 0)
    {
        success = UART_ParseFloat(line + 3, &value);
        if (success != 0U)
        {
            success = PID_SetKi(value);
        }
    }
    else if (strncmp(line, "Kd=", 3) == 0)
    {
        success = UART_ParseFloat(line + 3, &value);
        if (success != 0U)
        {
            success = PID_SetKd(value);
        }
    }
    else if (strncmp(line, "Base=", 5) == 0)
    {
        success = UART_ParseFloat(line + 5, &value);
        if (success != 0U)
        {
            success = PID_SetBaseSpeed(value);
        }
    }
    else if (strncmp(line, "LostExtra=", 10) == 0)
    {
        success = UART_ParseFloat(line + 10, &value);
        if (success != 0U)
        {
            success = PID_SetLostExtra(value);
        }
    }
    else if (strncmp(line, "LeftSpeed=", 10) == 0)
    {
        if (PID_IsRunning() != 0U)
        {
            UART_SendCommandResult(line, 0U);
            return;
        }
        /* 取 '=' 后面的数字，转成 uint32_t */
        speed = (uint32_t)strtoul(line + 10, NULL, 10);
        PWM_LeftSpeed(speed);
    }
    else if (strncmp(line, "RightSpeed=", 11) == 0)
    {
        if (PID_IsRunning() != 0U)
        {
            UART_SendCommandResult(line, 0U);
            return;
        }
        speed = (uint32_t)strtoul(line + 11, NULL, 10);
        PWM_RightSpeed(speed);
    }
    else
    {
        success = 0U;
    }

    UART_SendCommandResult(line, success);
}

static uint8_t UART_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return 0U;
    }
    parsed = strtof(text, &end);
    if ((end == text) || (*end != '\0') || (parsed != parsed))
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

static void UART_SendCommandResult(const char *line, uint8_t success)
{
    UART_SendString(line);
    UART_SendString((success != 0U) ? " OK\n" : " ERR\n");
}

static void UART_SendPidParameters(void)
{
    PID_Parameters parameters;
    uint32_t kp_milli;
    uint32_t ki_milli;
    uint32_t kd_milli;
    uint32_t base_milli;
    uint32_t lost_extra_milli;
    char line[128];

    PID_GetParameters(&parameters);
    kp_milli = (uint32_t)(parameters.kp * 1000.0f + 0.5f);
    ki_milli = (uint32_t)(parameters.ki * 1000.0f + 0.5f);
    kd_milli = (uint32_t)(parameters.kd * 1000.0f + 0.5f);
    base_milli = (uint32_t)(parameters.base_speed_us * 1000.0f + 0.5f);
    lost_extra_milli = (uint32_t)(parameters.lost_extra_us * 1000.0f + 0.5f);

    if (parameters.mode == PID_MODE_RIGHT_EDGE)
    {
        sprintf(line,
                "PID Mode=1 Kp=%lu.%03lu Ki=%lu.%03lu Kd=%lu.%03lu Base=%lu.%03lu LostExtra=%lu.%03lu Track=%u\n",
                (unsigned long)(kp_milli / 1000U),
                (unsigned long)(kp_milli % 1000U),
                (unsigned long)(ki_milli / 1000U),
                (unsigned long)(ki_milli % 1000U),
                (unsigned long)(kd_milli / 1000U),
                (unsigned long)(kd_milli % 1000U),
                (unsigned long)(base_milli / 1000U),
                (unsigned long)(base_milli % 1000U),
                (unsigned long)(lost_extra_milli / 1000U),
                (unsigned long)(lost_extra_milli % 1000U),
                (unsigned)parameters.running);
    }
    else
    {
        sprintf(line,
                "PID Mode=0 Kp=%lu.%03lu Ki=%lu.%03lu Kd=%lu.%03lu Base=%lu.%03lu Track=%u\n",
                (unsigned long)(kp_milli / 1000U),
                (unsigned long)(kp_milli % 1000U),
                (unsigned long)(ki_milli / 1000U),
                (unsigned long)(ki_milli % 1000U),
                (unsigned long)(kd_milli / 1000U),
                (unsigned long)(kd_milli % 1000U),
                (unsigned long)(base_milli / 1000U),
                (unsigned long)(base_milli % 1000U),
                (unsigned)parameters.running);
    }
    UART_SendString(line);
}

/**
  * @brief  红外状态有变化时，把新状态发到串口
  * @note   在主循环里调用，内部记着上次的值，只有变化才输出，静止时不发。
  *         原始位中 0 表示白线，1 表示黑底。
  *         输出格式：IR: 0xE7 11100111
  */
void UART_IR(void)
{
    static uint8_t last_state;
    static uint8_t initialized = 0;
    uint8_t state;
    char    bits[9];
    char    line[24];
    uint8_t i;

    if (ir_report_enabled == 0U)
    {
        return;
    }

    state = IR_Read();
    if ((initialized != 0U) && (state == last_state))
    {
        return;
    }
    initialized = 1;
    last_state = state;

    for (i = 0; i < 8; i++)             /* 从 bit7 开始，正好是左1~右1的顺序 */
    {
        bits[i] = ((state & (0x80 >> i)) != 0) ? '1' : '0';
    }
    bits[8] = '\0';

    sprintf(line, "IR: 0x%02X %s\r\n", (unsigned)state, bits);
    UART_SendString(line);
}
