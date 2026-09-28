/* ir.h */
#ifndef __IR_H
#define __IR_H

#include "main.h"

/**
  * @brief  读取 8 路红外传感器状态
  * @retval 8 位状态，bit7~bit0 依次对应从左到右 8 路，
  *         实测黑底为 1，白线为 0
  */
uint8_t IR_Read(void);
extern uint8_t ir_data[8];
void IR_Update(void);
extern volatile uint8_t ir_state;
/* ir.h */

#endif
