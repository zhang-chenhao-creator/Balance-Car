/***********************************************
 * 45度双折线避障：确认 -> 制动停车 -> 右45度移动弧线
 *                 -> 斜行 -> 左90度移动弧线 -> 搜线回巡
 * 固定向右绕行，不依赖路口或8字路线状态机。
 ***********************************************/
#ifndef __AVOID_ROUTINE_H
#define __AVOID_ROUTINE_H

#include "sys.h"
#include "obstacle_guard.h"

extern float Patrol_Speed_Cmd;
extern float Patrol_Turn_Cmd;

typedef enum
{
    AVOID_IDLE = 0,
    AVOID_BRAKE,
    AVOID_STOP_SETTLE,
    AVOID_ARC_RIGHT,
    AVOID_DIAGONAL,
    AVOID_ARC_LEFT,
    AVOID_SEARCH_LINE,
    AVOID_REENTER_LINE,
    AVOID_ABORT_HOLD
} AvoidState_t;

extern u8 Avoid_Active;
extern u8 Avoid_State;
extern u8 Avoid_Enable;
extern u8 Guard_State;

/* Keil Watch中可直接调整的首轮实车参数。 */
extern float Avoid_RightAngleDeg;
extern float Avoid_LeftAngleDeg;
extern float Avoid_DiagonalMm;
extern float Avoid_SearchMaxMm;
extern float Avoid_ArcSpeed;
extern float Avoid_SearchSpeed;
extern float Avoid_ReenterSpeed;
extern float Avoid_ArcTurnTarget;
extern float Avoid_BrakeFallStep;

void TrackAvoid_Init(void);
void TrackAvoid_Supervisor5ms(void);

#endif
