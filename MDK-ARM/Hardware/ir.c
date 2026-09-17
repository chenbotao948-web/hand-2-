/* ir.c */
#include "ir.h"

/**
  * @brief  读取 8 路红外传感器，拼成一个字节返回
  * @note   传感器从左到右依次对应 bit7 ~ bit0：
  *           bit7 -> 左1  PC14
  *           bit6 -> 左2  PC13
  *           bit5 -> 左3  PC12
  *           bit4 -> 左4  PC11
  *           bit3 -> 右4  PB12
  *           bit2 -> 右3  PB5
  *           bit1 -> 右2  PB4
  *           bit0 -> 右1  PB3
  *
  *         实测极性：黑底输出高电平，对应位为 1；
  *                   白线输出低电平，对应位为 0。
  *
  *         返回值示例：
  *           0x00 -> 8 路全部位于白色区域
  *           0x7F -> 仅最左一路位于白线
  *           0xE7 -> 中间两路位于白线
  *           0xFF -> 8 路全部位于黑底
  *
  * @retval 8 位状态，每位对应一路传感器
  */
uint8_t IR_Read(void)
{
    uint8_t state = 0;   /* 初始全 0，读到触发就置对应位 */

    /* 左1：PC14，bit7 */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET)
    {
        state |= 0x80;
    }

    /* 左2：PC13，bit6 */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
    {
        state |= 0x40;
    }

    /* 左3：PC12，bit5 */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET)
    {
        state |= 0x20;
    }

    /* 左4：PC11，bit4 */
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET)
    {
        state |= 0x10;
    }

    /* 右4：PB12，bit3 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET)
    {
        state |= 0x08;
    }

    /* 右3：PB5，bit2 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
    {
        state |= 0x04;
    }

    /* 右2：PB4，bit1 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET)
    {
        state |= 0x02;
    }

    /* 右1：PB3，bit0 */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET)
    {
        state |= 0x01;
    }

    return state;   /* 返回 8 路合并后的状态 */
}

/* ir.c */
uint8_t ir_data[8] = {0};   /* ir_data[0]~ir_data[7] 对应左1~右1 */

void IR_Update(void)
{
    uint8_t state = IR_Read();          /* 只读一次，存起来 */

    for (uint8_t i = 0; i < 8; i++)
    {
        ir_data[i] = (state >> (7 - i)) & 0x01;
    }
}





