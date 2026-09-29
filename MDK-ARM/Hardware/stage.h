#ifndef __STAGE_H
#define __STAGE_H

#include "main.h"

typedef enum
{
    STAGE_GO = 0,
	STAGE_CATCH,
    STAGE_STOP,
    STAGE_CENTER,
    STAGE_ANGLE,
    STAGE_EDGE
	
} Stage_State;

#define STAGE_DEFAULT_WAIT_SECONDS 20U//angle时间
#define STAGE_MAX_WAIT_SECONDS     5999U//最大时间
#define STAGE_DEFAULT_SPEED_TIME_SECONDS 25U//变速
#define STAGE_DEFAULT_CATCH_TIME_SECONDS 5U//开始抓时间
#define STAGE_DEFAULT_CATCH_DURATION_SECONDS 5U//持续时间

void Stage_Init(void);

void  Stage_Update(void);
extern uint32_t tim3_pid_tick_count;

void Stage_SetState(Stage_State state);
Stage_State Stage_GetState(void);
void Stage_StartRun(void);
void Stage_ResetRunTimer(void);
uint8_t Stage_SetWaitSeconds(uint16_t seconds);
uint8_t Stage_SetSpeedTimeSeconds(uint16_t seconds);
uint8_t Stage_SetCatchTimeSeconds(uint16_t seconds);
uint8_t Stage_SetCatchDurationSeconds(uint16_t seconds);
uint32_t Stage_GetElapsedSeconds(void);
#endif
