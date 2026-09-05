/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

说明：移植自 STM32F103RCT6_通用巡线例程_C10B / 椭圆巡线例程_C10B
  算法与参考例程一致（仅去掉了"赋值给两个电机目标速度"的语句，因为平衡
  小车的速度环/转向环独立，见 control.c 的 Velocity()/Turn()）
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
float ForwardLimit = 50;   // 前行限制(转向差速大于该值时前进速度降为0)（椭圆例程为80）
float Track_Turn_Scale = 0.7f; // turn_diff(mm/s) → 转向环目标幅值 的换算系数
                              // 参考:遥控全速转向时 Turn_Target≈54(即turn_diff≈77时)
float Track_Speed_RiseStep = 5;  // 每个5ms周期允许的加速步长（mm/s）
float Track_Speed_FallStep = 10; // 每个5ms周期允许的减速步长（mm/s）
u8 Track_CenterConfirmCycles = 3; // 恢复直行前需要连续确认的周期数
float Track_TurnAttackStep = 20;  // 紧急加大纠偏时每5ms最多增加的转向量
float Track_TurnReleaseStep = 10; // 减弱纠偏/换向回零时每5ms最多变化的转向量
u8 Track_TurnConfirmCycles = 2;   // 减弱或反向信号需连续确认，滤掉单帧跳变

/* 传感器状态定义见 TrackModule.h 中的 SensorState_t */

float base_speed_mm = 0;// 基础速度（mm/s）
float turn_diff = 0;    // 转向差速
u8 Track_state = 0;     // 最新识别的传感器状态(供OLED显示)

static int Track_SpeedState = STATE_STRAIGHT;
static u8 Track_CenterStableCount = 0;
static float Track_AcceptedTurn = 0;
static float Track_CandidateTurn = 0;
static u8 Track_TurnCandidateCount = 0;

static float Track_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static u8 Track_SameDirection(float left, float right)
{
    return ((left > 0.0f && right > 0.0f) ||
            (left < 0.0f && right < 0.0f)) ? 1U : 0U;
}

/*
 * 同向更强纠偏立即接受；减弱或换向必须稳定出现两帧。
 * 最终指令再做幅值斜坡，换向时先回到0，不跨零跳变。
 */
static float Track_FilterTurn(float current, float raw_target)
{
    float step;

    if (raw_target == Track_AcceptedTurn)
    {
        Track_CandidateTurn = raw_target;
        Track_TurnCandidateCount = 0;
    }
    else if ((Track_AcceptedTurn == 0.0f && raw_target != 0.0f) ||
             (Track_SameDirection(Track_AcceptedTurn, raw_target) &&
              Track_Abs(raw_target) > Track_Abs(Track_AcceptedTurn)))
    {
        Track_AcceptedTurn = raw_target;
        Track_CandidateTurn = raw_target;
        Track_TurnCandidateCount = 0;
    }
    else
    {
        if (raw_target == Track_CandidateTurn)
        {
            if (Track_TurnCandidateCount < Track_TurnConfirmCycles)
                Track_TurnCandidateCount++;
        }
        else
        {
            Track_CandidateTurn = raw_target;
            Track_TurnCandidateCount = 1;
        }

        if (Track_TurnCandidateCount >= Track_TurnConfirmCycles)
        {
            Track_AcceptedTurn = raw_target;
            Track_TurnCandidateCount = 0;
        }
    }

    if (current == Track_AcceptedTurn)
        return current;

    if (current * Track_AcceptedTurn < 0.0f)
    {
        if (Track_Abs(current) <= Track_TurnReleaseStep)
            return 0.0f;
        return current + ((current > 0.0f) ?
                          -Track_TurnReleaseStep : Track_TurnReleaseStep);
    }

    step = (Track_Abs(Track_AcceptedTurn) > Track_Abs(current)) ?
           Track_TurnAttackStep : Track_TurnReleaseStep;
    if (Track_AcceptedTurn > current + step)
        return current + step;
    if (Track_AcceptedTurn < current - step)
        return current - step;
    return Track_AcceptedTurn;
}

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

/*=============================================================================*
 * 巡线功能函数（计算 base_speed_mm / turn_diff 两个输出量）					   *
 * 在5ms中断中调用															   *
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;// 记录上一次的状态
    float raw_turn = Track_AcceptedTurn;

    // 读取传感器状态：4个传感器组合值
    int sensor_state = (DH1 << 3) | (DH2 << 2) | (DH3 << 1) | DH4;
    Track_state = sensor_state;   // 供OLED显示当前识别状态
    /*=========================================================================*
     * 状态判断：设置转向差速												   *
     *=========================================================================*/
    switch (sensor_state)
    {
       case STATE_CROSS:// 交叉路口处理
			raw_turn = 0;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            raw_turn = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            raw_turn = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://左大弯
            raw_turn = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://右大弯
            raw_turn = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://左微调
            raw_turn = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://右微调
            raw_turn = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直行
            raw_turn = 0;
            break;
        case STATE_LOST://丢线处理
			if (last_state == STATE_LEFT_SMALL) raw_turn = TurnMidAngle;//继续左转
			else if (last_state == STATE_RIGHT_SMALL) raw_turn = -TurnMidAngle;//继续右转
			else if(last_state == STATE_LEFT_BIG ) raw_turn = TurnMaxAngle;//继续左转
			else if(last_state == STATE_RIGHT_BIG ) raw_turn = -TurnMaxAngle;//继续右转
			//其余情况维持上一次的转向(与参考例程一致)
            break;
        default: // 未定义状态，直行
            raw_turn = 0;
            break;
    }
	//保存传感器状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
    turn_diff = Track_FilterTurn(turn_diff, raw_turn);
    /*
     * 速度不再随状态直接跳变：先得到离散目标，再通过斜坡限制实际速度。
     * 1001 恢复到直道时需要连续确认，进入弯道或异常状态则立即降目标。
     */
    base_speed_mm = Track_Speed_Ramp(
        base_speed_mm,
        Track_TargetSpeedForState(Track_GetSpeedState(sensor_state)));
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
	base_speed_mm = 0;
	turn_diff = 0;
	Track_state = 0;
	Track_SpeedState = STATE_STRAIGHT;
	Track_CenterStableCount = 0;
	Track_AcceptedTurn = 0;
	Track_CandidateTurn = 0;
	Track_TurnCandidateCount = 0;

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
