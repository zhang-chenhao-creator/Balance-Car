/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

说明：移植自 STM32F103RCT6_通用巡线例程_C10B / 椭圆巡线例程_C10B。
  日常巡线仍输出 base_speed_mm / turn_diff 给平衡车速度环和转向环。
  本文件增加固定路线状态机和按最后位置回搜的丢线恢复。
All rights reserved
***********************************************/
#include "TrackModule.h"

/*=============================================================================*
 * 可调参数区域																   *
 * 默认使用"通用巡线"参数；"椭圆巡线"参数见注释，可自行切换					   *
 *=============================================================================*/
float Turn90Angle  = 80;   // 直角弯转向参数
float TurnMaxAngle = 65;   // 大弯道转向参数（椭圆例程为45）
float TurnMidAngle = 40;   // 中等转向参数（丢线时使用）（椭圆例程为25）
float TurnMinAngle = 15;   // 微调转向参数
float BaseSpeed = 400;     // 直道基础巡线速度（mm/s），降低直道进入弯道时的惯性
float FineSpeed = 375;     // 微调状态目标速度（mm/s）
float CurveSpeed = 350;    // 普通弯道目标速度（mm/s）
float BigCurveSpeed = 325; // 大弯道目标速度（mm/s）
float LostSpeed = 200;     // 丢线或未定义状态的安全速度（mm/s）
float JunctionSpeed = 200; // 路口转向用现有最慢档，日常巡线不被锁成0
float FinishSpeed = 200;   // 最后一段接近终点：用现有最慢档
float ForwardLimit = 50;   // 前行限制(转向差速大于该值时前进速度降为0)（椭圆例程为80）
float Track_Turn_Scale = 0.7f; // turn_diff(mm/s) → 转向环目标幅值 的换算系数
                              // 参考:遥控全速转向时 Turn_Target≈54(即turn_diff≈77时)
float Track_Speed_RiseStep = 5;  // 每个5ms周期允许的加速步长（mm/s）
float Track_Speed_FallStep = 10; // 每个5ms周期允许的减速步长（mm/s）
u8 Track_CenterConfirmCycles = 3; // 恢复直行前需要连续确认的周期数

#define TRACK_START_CENTER_CYCLES    6   /* 30ms */
#define TRACK_FORK_DEBOUNCE          3   /* 15ms，避免直道偏线误当路口 */
#define TRACK_CROSS_DEBOUNCE         3   /* 15ms */
#define TRACK_TURN_MIN_CYCLES       30   /* 150ms */
#define TRACK_TURN_CENTER_CYCLES     3   /* 15ms */
#define TRACK_TURN_WIDE_CYCLES      80   /* 400ms，宽胶带时允许不出现中间全白 */
#define TRACK_TURN_TIMEOUT_CYCLES  240   /* 1.2s */
#define TRACK_LOST_DEBOUNCE          3   /* 15ms */
#define TRACK_RECOVER_CYCLES         3   /* 15ms */
#define TRACK_SEARCH_TIMEOUT_CYCLES 240  /* 1.2s */

#define TRACK_ROUTE_LEN 5

typedef enum {
    RUN_WAIT_START = 0,
    RUN_PATROL,
    RUN_TURN,
    RUN_SEARCH,
    RUN_FINISH,
    RUN_FAULT
} TrackRun_t;

typedef enum {
    ACT_LEFT = 0,
    ACT_RIGHT,
    ACT_STOP
} TrackAct_t;

typedef enum {
    TRIG_LEFT_FORK = 0,
    TRIG_RIGHT_FORK,
    TRIG_CROSS
} TrackTrig_t;

typedef enum {
    SIDE_CENTER = 0,
    SIDE_LEFT,
    SIDE_RIGHT
} TrackSide_t;

typedef struct {
    TrackTrig_t trig;
    TrackAct_t act;
} TrackRouteItem_t;

/* 固定赛道：左岔口左转、全黑口右转、右岔口右转、全黑口左转、全黑口停车 */
static const TrackRouteItem_t Track_Route[TRACK_ROUTE_LEN] = {
    { TRIG_LEFT_FORK,  ACT_LEFT },
    { TRIG_CROSS,      ACT_RIGHT },
    { TRIG_RIGHT_FORK, ACT_RIGHT },
    { TRIG_CROSS,      ACT_LEFT },
    { TRIG_CROSS,      ACT_STOP }
};

float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速
u8 Track_state = 0;     // 最新识别的传感器状态(供OLED显示)

static int Track_SpeedState = STATE_STRAIGHT;
static u8 Track_CenterStableCount = 0;

static TrackRun_t Track_Run = RUN_WAIT_START;
static u8 Track_RouteIndex = 0;
static TrackAct_t Track_LockedAct = ACT_LEFT;
static TrackSide_t Track_LastSide = SIDE_CENTER;
static u8 Track_Armed = 0;
static u8 Track_CenterHold = 0;
static u8 Track_JunctionCount = 0;
static u8 Track_LostCount = 0;
static u8 Track_RecoverCount = 0;
static u16 Track_TurnCycles = 0;
static u8 Track_TurnSawGap = 0;
static u8 Track_TurnCenterCount = 0;
static u16 Track_SearchCycles = 0;

/*=============================================================================*
 * 速度目标与实际下发值之间的斜坡限制                                     *
 * 减速步长大于加速步长，保证进弯及时降速、出弯平滑恢复。                 *
 *=============================================================================*/
static float Track_Speed_Ramp(float current, float target)
{
    float step;

    if (target > current)
    {
        step = Track_Speed_RiseStep;
        if (target - current <= step) return target;
        return current + step;
    }

    step = Track_Speed_FallStep;
    if (current - target <= step) return target;
    return current - step;
}

/* 直道恢复需要连续确认，进入弯道则立即采用更低的速度目标。 */
static int Track_GetSpeedState(int sensor_state)
{
    if (sensor_state == STATE_STRAIGHT)
    {
        if (Track_SpeedState != STATE_STRAIGHT)
        {
            if (Track_CenterStableCount < Track_CenterConfirmCycles)
            {
                Track_CenterStableCount++;
            }
            if (Track_CenterStableCount >= Track_CenterConfirmCycles)
            {
                Track_SpeedState = STATE_STRAIGHT;
            }
        }
    }
    else
    {
        Track_CenterStableCount = 0;
        Track_SpeedState = sensor_state;
    }

    return Track_SpeedState;
}

static float Track_TargetSpeedForState(int sensor_state)
{
    switch (sensor_state)
    {
        case STATE_STRAIGHT:
            return BaseSpeed;
        case STATE_LEFT_SMALL:
        case STATE_RIGHT_SMALL:
            return FineSpeed;
        case STATE_LEFT_90_A:
        case STATE_LEFT_90_B:
        case STATE_RIGHT_90_A:
        case STATE_RIGHT_90_B:
            return CurveSpeed;
        case STATE_LEFT_BIG:
        case STATE_RIGHT_BIG:
        case STATE_CROSS:
            return BigCurveSpeed;
        case STATE_LOST:
        default:
            return LostSpeed;
    }
}

static int Track_IsConfirmedLeftFork(int sensor_state)
{
    /* 主线还在中间、左外新出现黑线：真正的左岔口 */
    return sensor_state == STATE_RIGHT_90_A;
}

static int Track_IsLeftFork(int sensor_state)
{
    /* 只认左三路压线(1000)和左两路压线(1100)。
     * 只有左外(1110)是普通大偏差，必须继续当巡线纠偏，不能锁成路口。 */
    return Track_IsConfirmedLeftFork(sensor_state) ||
           (sensor_state == STATE_RIGHT_90_B);
}

static int Track_IsConfirmedRightFork(int sensor_state)
{
    return sensor_state == STATE_LEFT_90_A;
}

static int Track_IsRightFork(int sensor_state)
{
    /* 只认白黑黑黑(0001)。白白黑黑(0011)仍当右偏，避免椭圆弯被当成右岔口。 */
    return Track_IsConfirmedRightFork(sensor_state);
}

static int Track_IsConfirmedFork(int sensor_state, TrackTrig_t trig)
{
    if (trig == TRIG_LEFT_FORK) return Track_IsConfirmedLeftFork(sensor_state);
    if (trig == TRIG_RIGHT_FORK) return Track_IsConfirmedRightFork(sensor_state);
    /* 全黑(抬车四路灯全亮)必须先居中过，不能直接当路口/终点 */
    return 0;
}

static u8 Track_NeededDebounce(TrackTrig_t trig)
{
    if (trig == TRIG_CROSS) return TRACK_CROSS_DEBOUNCE;
    return TRACK_FORK_DEBOUNCE;
}

static int Track_MatchesTrig(int sensor_state, TrackTrig_t trig)
{
    if (trig == TRIG_LEFT_FORK) return Track_IsLeftFork(sensor_state);
    if (trig == TRIG_RIGHT_FORK) return Track_IsRightFork(sensor_state);
    return sensor_state == STATE_CROSS;
}

static TrackSide_t Track_SideFromSensor(int sensor_state)
{
    if (Track_IsLeftFork(sensor_state) ||
        (sensor_state == STATE_RIGHT_BIG) ||
        (sensor_state == STATE_RIGHT_SMALL))
    {
        return SIDE_LEFT;
    }
    if (Track_IsRightFork(sensor_state) ||
        (sensor_state == STATE_LEFT_90_B) ||
        (sensor_state == STATE_LEFT_BIG) ||
        (sensor_state == STATE_LEFT_SMALL))
    {
        return SIDE_RIGHT;
    }
    return SIDE_CENTER;
}

static int Track_BothMidsOnLine(int sensor_state)
{
    /* DH2=bit2、DH3=bit1，黑线为0：两中都压线时这两位都是0 */
    return (sensor_state & 0x06) == 0;
}

static int Track_CaughtNewLine(int sensor_state, TrackAct_t act)
{
    if (sensor_state == STATE_STRAIGHT) return 1;
    if (act == ACT_LEFT)
    {
        return (sensor_state == STATE_RIGHT_SMALL) ||
               (sensor_state == STATE_RIGHT_90_A) ||
               (sensor_state == STATE_RIGHT_90_B) ||
               (sensor_state == STATE_RIGHT_BIG);
    }
    if (act == ACT_RIGHT)
    {
        return (sensor_state == STATE_LEFT_SMALL) ||
               (sensor_state == STATE_LEFT_90_A) ||
               (sensor_state == STATE_LEFT_90_B) ||
               (sensor_state == STATE_LEFT_BIG);
    }
    return 0;
}

static void Track_FollowPatrol(int sensor_state)
{
    switch (sensor_state)
    {
        case STATE_CROSS:
            turn_diff = 0;
            break;
        case STATE_LEFT_90_A:
        case STATE_LEFT_90_B:
            turn_diff = Turn90Angle; /* 物理右转 */
            break;
        case STATE_RIGHT_90_A:
        case STATE_RIGHT_90_B:
            turn_diff = -Turn90Angle; /* 物理左转 */
            break;
        case STATE_LEFT_BIG:
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG:
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL:
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL:
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT:
            turn_diff = 0;
            break;
        default:
            turn_diff = 0;
            break;
    }
}

static void Track_ApplyLockedTurn(TrackAct_t act)
{
    if (act == ACT_LEFT) turn_diff = -Turn90Angle;
    else if (act == ACT_RIGHT) turn_diff = Turn90Angle;
    else turn_diff = 0;
}

static void Track_ApplySearchTurn(void)
{
    if (Track_LastSide == SIDE_LEFT) turn_diff = -TurnMaxAngle;
    else if (Track_LastSide == SIDE_RIGHT) turn_diff = TurnMaxAngle;
    else turn_diff = 0;
}

static void Track_ResetTurnFlags(void)
{
    Track_TurnCycles = 0;
    Track_TurnSawGap = 0;
    Track_TurnCenterCount = 0;
}

static void Track_LockRouteAction(void)
{
    Track_LockedAct = Track_Route[Track_RouteIndex].act;
    Track_RouteIndex++;
    Track_JunctionCount = 0;
    Track_Armed = 0;
    Track_CenterHold = 0;
    Track_LostCount = 0;
    if (Track_LockedAct == ACT_STOP)
    {
        Track_Run = RUN_FINISH;
        turn_diff = 0;
        return;
    }
    Track_Run = RUN_TURN;
    Track_ResetTurnFlags();
    Track_ApplyLockedTurn(Track_LockedAct);
}

static void Track_UpdateArming(int sensor_state)
{
    TrackTrig_t trig;

    if (sensor_state == STATE_STRAIGHT)
    {
        if (Track_CenterHold < 255) Track_CenterHold++;
        if (Track_CenterHold >= TRACK_START_CENTER_CYCLES) Track_Armed = 1;
        Track_JunctionCount = 0;
        return;
    }

    if (Track_RouteIndex >= TRACK_ROUTE_LEN)
    {
        Track_JunctionCount = 0;
        return;
    }

    trig = Track_Route[Track_RouteIndex].trig;
    if (Track_MatchesTrig(sensor_state, trig))
    {
        /* 明确左/右岔口(中间主线还在)可直接认；全黑口和 1100 必须先居中。 */
        if (Track_Armed || Track_IsConfirmedFork(sensor_state, trig))
        {
            if (Track_JunctionCount < 255) Track_JunctionCount++;
        }
        else if (Track_JunctionCount > 0)
        {
            Track_JunctionCount--;
        }
        else
        {
            Track_JunctionCount = 0;
        }
        return;
    }

    if (Track_JunctionCount > 0)
    {
        Track_JunctionCount--;
        return;
    }
    if (!Track_IsLeftFork(sensor_state) && !Track_IsRightFork(sensor_state))
    {
        Track_Armed = 0;
        Track_CenterHold = 0;
    }
}

static void Track_EnterSearch(void)
{
    Track_Run = RUN_SEARCH;
    Track_SearchCycles = 0;
    Track_RecoverCount = 0;
    Track_LostCount = 0;
    Track_JunctionCount = 0;
    Track_ApplySearchTurn();
}

static void Track_EnterFault(void)
{
    Track_Run = RUN_FAULT;
    turn_diff = 0;
}

static float Track_PatrolTargetSpeed(int sensor_state)
{
    if (Track_RouteIndex >= (TRACK_ROUTE_LEN - 1)) return FinishSpeed;
    return Track_TargetSpeedForState(Track_GetSpeedState(sensor_state));
}

static void Track_RouteStep(int sensor_state)
{
    float target_speed;

    if (sensor_state != STATE_LOST)
    {
        Track_LastSide = Track_SideFromSensor(sensor_state);
        Track_LostCount = 0;
    }
    else if (Track_LostCount < 255)
    {
        Track_LostCount++;
    }

    switch (Track_Run)
    {
        case RUN_WAIT_START:
            if (sensor_state == STATE_LOST)
            {
                if (Track_LostCount >= TRACK_LOST_DEBOUNCE) Track_EnterSearch();
                target_speed = LostSpeed;
                break;
            }
            Track_FollowPatrol(sensor_state);
            if (sensor_state == STATE_STRAIGHT)
            {
                if (Track_CenterHold < 255) Track_CenterHold++;
            }
            else
            {
                Track_CenterHold = 0;
            }
            if ((sensor_state != STATE_CROSS) &&
                (Track_CenterHold >= TRACK_START_CENTER_CYCLES))
            {
                Track_Run = RUN_PATROL;
                Track_Armed = 1;
            }
            target_speed = Track_TargetSpeedForState(Track_GetSpeedState(sensor_state));
            break;

        case RUN_PATROL:
            if (sensor_state == STATE_LOST)
            {
                if (Track_LostCount >= TRACK_LOST_DEBOUNCE) Track_EnterSearch();
                target_speed = LostSpeed;
                break;
            }
            Track_FollowPatrol(sensor_state);
            Track_UpdateArming(sensor_state);
            if ((Track_RouteIndex < TRACK_ROUTE_LEN) &&
                (Track_JunctionCount >= Track_NeededDebounce(Track_Route[Track_RouteIndex].trig)))
            {
                Track_LockRouteAction();
                if (Track_Run == RUN_FINISH) target_speed = 0;
                else target_speed = JunctionSpeed;
                break;
            }
            target_speed = Track_PatrolTargetSpeed(sensor_state);
            break;

        case RUN_TURN:
            Track_ApplyLockedTurn(Track_LockedAct);
            if (Track_TurnCycles < 65535) Track_TurnCycles++;
            if (!Track_BothMidsOnLine(sensor_state) || (sensor_state == STATE_LOST))
            {
                Track_TurnSawGap = 1;
            }
            if (Track_TurnCycles >= TRACK_TURN_TIMEOUT_CYCLES)
            {
                if (Track_LockedAct == ACT_LEFT) Track_LastSide = SIDE_LEFT;
                else if (Track_LockedAct == ACT_RIGHT) Track_LastSide = SIDE_RIGHT;
                Track_EnterSearch();
                target_speed = JunctionSpeed;
                break;
            }
            if ((Track_TurnCycles >= TRACK_TURN_MIN_CYCLES) &&
                (Track_TurnSawGap || (Track_TurnCycles >= TRACK_TURN_WIDE_CYCLES)) &&
                Track_CaughtNewLine(sensor_state, Track_LockedAct))
            {
                if (Track_TurnCenterCount < 255) Track_TurnCenterCount++;
            }
            else
            {
                Track_TurnCenterCount = 0;
            }
            if (Track_TurnCenterCount >= TRACK_TURN_CENTER_CYCLES)
            {
                Track_Run = RUN_PATROL;
                Track_ResetTurnFlags();
                Track_Armed = 0;
                Track_CenterHold = 0;
                Track_FollowPatrol(sensor_state);
            }
            target_speed = JunctionSpeed;
            break;

        case RUN_SEARCH:
            Track_ApplySearchTurn();
            if (Track_SearchCycles < 65535) Track_SearchCycles++;
            if (sensor_state != STATE_LOST)
            {
                if (Track_RecoverCount < 255) Track_RecoverCount++;
            }
            else
            {
                Track_RecoverCount = 0;
            }
            if (Track_RecoverCount >= TRACK_RECOVER_CYCLES)
            {
                Track_Run = RUN_PATROL;
                Track_SearchCycles = 0;
                Track_RecoverCount = 0;
                Track_FollowPatrol(sensor_state);
                target_speed = Track_PatrolTargetSpeed(sensor_state);
                break;
            }
            if (Track_SearchCycles >= TRACK_SEARCH_TIMEOUT_CYCLES)
            {
                Track_EnterFault();
                target_speed = 0;
                break;
            }
            /* 偏左/偏右丢线原地转着找，居中缺口才低速直行跨越 */
            if (Track_LastSide == SIDE_CENTER) target_speed = LostSpeed;
            else target_speed = JunctionSpeed;
            break;

        case RUN_FINISH:
        case RUN_FAULT:
        default:
            turn_diff = 0;
            target_speed = 0;
            break;
    }

    base_speed_mm = Track_Speed_Ramp(base_speed_mm, target_speed);
}

/*=============================================================================*
 * 巡线功能函数（计算 base_speed_mm / turn_diff 两个输出量）					   *
 * 在5ms中断中调用															   *
 *=============================================================================*/
void Track_ResetLogic(void)
{
    base_speed_mm = 0;
    turn_diff = 0;
    Track_state = 0;
    Track_SpeedState = STATE_STRAIGHT;
    Track_CenterStableCount = 0;
    Track_Run = RUN_WAIT_START;
    Track_RouteIndex = 0;
    Track_LockedAct = ACT_LEFT;
    Track_LastSide = SIDE_CENTER;
    Track_Armed = 0;
    Track_CenterHold = 0;
    Track_JunctionCount = 0;
    Track_LostCount = 0;
    Track_RecoverCount = 0;
    Track_TurnCycles = 0;
    Track_TurnSawGap = 0;
    Track_TurnCenterCount = 0;
    Track_SearchCycles = 0;
}

void IRDM_line_inspection(void)
{
    int sensor_state;

    /* 抬车时四路红外都看不到白底，会变成全黑(0000)。
     * 不能当路口/终点，复位后落地再从普通巡线开始。 */
    if (Pick_up_stop)
    {
        Track_ResetLogic();
        return;
    }

    sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
    Track_state = sensor_state;
    Track_RouteStep(sensor_state);
}

/**************************************************************************
Function: 进入巡线模式时的引脚配置
Input   : none
Output  : none
函数功能：把扩展接口(PB8/PC8/PC4/PC9)配置为4路巡线传感器输入(下拉)。
          PS2手柄与巡线模块共用该接口，两者不能同时使用。
入口参数：无
返回  值：无
**************************************************************************/
void TrackModule_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	Track_ResetLogic();

	__HAL_RCC_GPIOC_CLK_ENABLE();      // 使能 GPIOC 时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();      // 使能 GPIOB 时钟

	// 配置 GPIOC 的 8、4、9 引脚为输入下拉模式 (DH1/DH2/DH3)
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_4 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 配置 GPIOB 的 8 引脚为输入下拉模式 (DH4)
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**************************************************************************
Function: 退出巡线模式时的引脚恢复
Input   : none
Output  : none
函数功能：恢复扩展接口为PS2手柄默认配置(与gpio.c中MX_GPIO_Init一致)：
          PC4/PC8/PC9 推挽输出高、PB8 输入上拉。
入口参数：无
返回  值：无
**************************************************************************/
void TrackModule_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// PS2: DO=PC9  CS=PC4  CLK=PC8  (推挽输出, 初始高)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_4 | GPIO_PIN_9, GPIO_PIN_SET);
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_4 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PS2: DI=PB8 (输入上拉)
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}
