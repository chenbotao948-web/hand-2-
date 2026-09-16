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
static uint16_t rx_index = 0;                  /* rx_buffer 当前写入位置 */
static volatile uint8_t rx_line_ready = 0;     /* 1 表示有一条完整命令等待主循环处理 */

static void UART_StartReceive(void);
static void UART_ProcessLine(char *line);//程序对比表格

/**
  * @brief  串口初始化，启动第一次中断接收
  * @note   在 main 的 USER CODE BEGIN 2 中调用，
  *         必须在 MX_USART1_UART_Init() 之后。
  */
void UART_Init(void)
{
    rx_index = 0;//数组位置为零
    rx_line_ready = 0;
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

    UART_ProcessLine(rx_buffer);
    rx_line_ready = 0;
}

/**
  * @brief  串口接收完成回调，每收到 1 字节进一次
  * @param  huart: 触发回调的串口句柄
  * @note   做法 A：
  *         - 上一条命令还没被主循环处理完时，当前字节直接丢弃；
  *         - 收到 '\n' 表示一行结束，通知主循环处理；
  *         - '\r' 直接忽略，兼容 "\r\n" 结尾。
  *         代价：发太快时可能丢命令，也可能出现半行，
  *         适合手动一条条发送、发一条等一条的场景。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 上一条还没处理完，当前字节直接丢弃 */
        if (rx_line_ready != 0)
        {
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
            if (rx_index < (UART_RX_BUFFER_SIZE - 1))
            {
                rx_buffer[rx_index++] = rx_byte;
            }
            /* 缓冲区满，静默丢弃后续字符，等 '\n' 封口 */
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
        UART_StartReceive();
    }
}

/**
  * @brief  解析一行命令并执行
  * @param  line: 以 '\0' 结尾的一行命令字符串
  * @note   支持 "LeftSpeed=数值"、"RightSpeed=数值"、
  *         "Track=1" 和 "Track=0"。
  *         成功执行后回显 "原命令 OK\n"；否则回显 "原命令 ERR\n"。
  */
static void UART_ProcessLine(char *line)
{
    uint32_t speed;

    if (strcmp(line, "Track=1") == 0)
    {
        PID_Start();
    }
    else if (strcmp(line, "Track=0") == 0)
    {
        PID_Stop();
    }
    else if (strncmp(line, "LeftSpeed=", 10) == 0)
    {
        if (PID_IsRunning() != 0U)
        {
            UART_SendString(line);
            UART_SendString(" ERR\n");
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
            UART_SendString(line);
            UART_SendString(" ERR\n");
            return;
        }
        speed = (uint32_t)strtoul(line + 11, NULL, 10);
        PWM_RightSpeed(speed);
    }
    else
    {
        /* 无法识别的命令 */
        UART_SendString(line);
        UART_SendString(" ERR\n");
        return;
    }

    /* 回显：原命令 + " OK\n" */
    UART_SendString(line);
    UART_SendString(" OK\n");
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
