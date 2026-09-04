/***********************************************
 * 巡线避障：停车 -> 右转90 -> D1 -> 左转90 -> D2
 *           -> 左转90 朝线 -> 找线 -> 右转对准 -> 低速回巡
 * 固定向右绕。不依赖 8 字路口状态机。
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
    AVOID_STOP_SETTLE,
    AVOID_TURN1_RIGHT,
    AVOID_LEG1,
    AVOID_TURN2_LEFT,
    AVOID_LEG2,
    AVOID_TURN3_LEFT,
    AVOID_SEEK_LINE,
    AVOID_ALIGN_RIGHT,
    AVOID_REJOIN_FOLLOW,
    AVOID_ABORT_HOLD
} AvoidState_t;

extern u8 Avoid_Active;
extern u8 Avoid_State;
extern u8 Avoid_Enable;
extern u8 Guard_State;

extern float Avoid_D1_mm;
extern float Avoid_D2_mm;
extern float Avoid_SeekMax_mm;
extern float Avoid_LegSpeed;
extern float Avoid_SeekSpeed;
extern float Avoid_TurnTarget;
extern float Avoid_TurnAngle;

void TrackAvoid_Init(void);
void TrackAvoid_Supervisor5ms(void);

#endif
