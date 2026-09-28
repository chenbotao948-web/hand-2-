#ifndef __STAGE_H
#define __STAGE_H

#include "main.h"

typedef enum
{
    STAGE_GO = 0,
    STAGE_STOP,
    STAGE_CENTER,
    STAGE_ANGLE,
    STAGE_EDGE
	
} Stage_State;

#define STAGE_DEFAULT_WAIT_SECONDS 20U
#define STAGE_MAX_WAIT_SECONDS     5999U

void Stage_Init(void);

void  Stage_Update(void);
extern uint32_t tim3_pid_tick_count;

void Stage_SetState(Stage_State state);
Stage_State Stage_GetState(void);
void Stage_StartRun(void);
void Stage_ResetRunTimer(void);
uint8_t Stage_SetWaitSeconds(uint16_t seconds);
uint32_t Stage_GetElapsedSeconds(void);
#endif
