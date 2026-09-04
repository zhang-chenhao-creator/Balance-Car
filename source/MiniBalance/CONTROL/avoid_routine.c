#include "avoid_routine.h"
#include "obstacle_guard.h"
#include "ultrasonic_service.h"
#include "TrackModule.h"
#include "control.h"
#include "show.h"

extern int   Encoder_Left, Encoder_Right;
extern float Gyro_Turn;
extern u8    Lidar_Detect;

#define ENCODER_TO_MM             (Perimeter / EncoderMultiples / Reduction_Ratio / Encoder_precision)
#define GYRO_Z_TO_DPS             (1.0f / 16.4f)
#define CONTROL_PERIOD_S          0.005f

#define AVOID_STOP_SETTLE_TICKS   80U    /* 400ms */
#define AVOID_BRAKE_TIMEOUT       200U   /* 1s */
#define AVOID_BRAKE_STABLE_TICKS  6U     /* 30ms */
#define AVOID_STOP_SPEED_MM_S     50.0f
#define AVOID_TURN_TIMEOUT        1000U  /* 5s */
#define AVOID_LINE_CONFIRM        3U
#define AVOID_REENTER_CONFIRM     6U
#define AVOID_REENTER_MIN_TICKS   40U    /* 200ms */
#define AVOID_REENTER_TIMEOUT     240U   /* 1.2s */
#define AVOID_COOLDOWN_TICKS      400U   /* 2s */
#define AVOID_SPEED_RISE_STEP     5.0f

float Patrol_Speed_Cmd = 0;
float Patrol_Turn_Cmd  = 0;
u8 Avoid_Active = 0;
u8 Avoid_State  = AVOID_IDLE;
u8 Avoid_Enable = 1;
u8 Guard_State  = OBSTACLE_GUARD_CLEAR;

float Avoid_RightAngleDeg = 45.0f;
float Avoid_LeftAngleDeg  = 90.0f;
float Avoid_DiagonalMm    = 300.0f;
float Avoid_SearchMaxMm   = 800.0f;
float Avoid_ArcSpeed      = 120.0f;
float Avoid_SearchSpeed   = 100.0f;
float Avoid_ReenterSpeed  = 120.0f;
float Avoid_ArcTurnTarget = 27.0f;
float Avoid_BrakeFallStep = 10.0f;

static ObstacleGuardContext s_guard;
static u8    s_guard_enabled_prev = 0xFF;
static u8    s_route_guard_armed;
static u8    s_line_confirm;
static u8    s_center_confirm;
static u8    s_stop_confirm;
static u16   s_state_ticks;
static u16   s_cooldown;
static float s_odom_mm;
static float s_turn_deg;
static float s_avoid_speed;
static float s_resume_speed;

static float Avoid_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Avoid_Ramp(float current, float target, float rise_step, float fall_step)
{
    float step = (target > current) ? rise_step : fall_step;

    if (target > current + step)
        return current + step;
    if (target < current - step)
        return current - step;
    return target;
}

static void Avoid_EnterState(u8 next)
{
    Avoid_State = next;
    s_state_ticks = 0;
    s_odom_mm = 0;
    s_turn_deg = 0;
    s_line_confirm = 0;
    s_center_confirm = 0;
    s_stop_confirm = 0;
}

static float Avoid_MeasuredSpeed(void)
{
    return (Avoid_Abs(Velocity_Left) + Avoid_Abs(Velocity_Right)) * 0.5f;
}

static void Avoid_AccumulateMotion(void)
{
    s_odom_mm += (Avoid_Abs((float)Encoder_Left) +
                  Avoid_Abs((float)Encoder_Right)) * 0.5f * ENCODER_TO_MM;
    s_turn_deg += Avoid_Abs(Gyro_Turn) * GYRO_Z_TO_DPS * CONTROL_PERIOD_S;
}

static u8 Avoid_DistanceDone(float target_mm, float speed_mm_s)
{
    u32 timeout_ticks;

    if (s_odom_mm >= target_mm)
        return 1U;

    if (speed_mm_s < 1.0f)
        speed_mm_s = 1.0f;
    timeout_ticks = (u32)(target_mm / speed_mm_s * 200.0f * 2.5f) + 200U;
    if (s_state_ticks >= timeout_ticks)
        return 2U;
    return 0U;
}

static u8 Avoid_AngleDone(float target_deg)
{
    if (s_turn_deg >= target_deg)
        return 1U;
    if (s_state_ticks >= AVOID_TURN_TIMEOUT)
        return 2U;
    return 0U;
}

static u8 Avoid_OnLine(void)
{
    return (Track_state != STATE_LOST) ? 1U : 0U;
}

static u8 Avoid_NearCenter(void)
{
    return (Track_state == STATE_STRAIGHT ||
            Track_state == STATE_LEFT_SMALL ||
            Track_state == STATE_RIGHT_SMALL) ? 1U : 0U;
}

static void Avoid_Abort(void)
{
    Avoid_EnterState(AVOID_ABORT_HOLD);
}

static void AvoidRoutine_Update5ms(u8 obstacle_entered)
{
    u8 done;

    s_state_ticks++;
    if (Avoid_Active)
        Avoid_AccumulateMotion();

    switch (Avoid_State)
    {
    case AVOID_IDLE:
        Avoid_Active = 0;
        if (s_cooldown > 0U)
            s_cooldown--;
        if (obstacle_entered && Avoid_Enable && s_cooldown == 0U)
        {
            Avoid_Active = 1;
            s_route_guard_armed = 0;
            s_avoid_speed = Patrol_Speed_Cmd;
            Avoid_EnterState(AVOID_BRAKE);
        }
        break;

    case AVOID_BRAKE:
        if (Patrol_Speed_Cmd <= 0.5f && Avoid_MeasuredSpeed() <= AVOID_STOP_SPEED_MM_S)
        {
            if (s_stop_confirm < AVOID_BRAKE_STABLE_TICKS)
                s_stop_confirm++;
        }
        else
        {
            s_stop_confirm = 0;
        }
        if (s_stop_confirm >= AVOID_BRAKE_STABLE_TICKS ||
            s_state_ticks >= AVOID_BRAKE_TIMEOUT)
            Avoid_EnterState(AVOID_STOP_SETTLE);
        break;

    case AVOID_STOP_SETTLE:
        if (s_state_ticks >= AVOID_STOP_SETTLE_TICKS)
        {
            ObstacleGuard_SetEnabled(&s_guard, 1U);
            Guard_State = OBSTACLE_GUARD_CLEAR;
            Avoid_EnterState(AVOID_ARC_RIGHT);
        }
        break;

    case AVOID_ARC_RIGHT:
        done = Avoid_AngleDone(Avoid_RightAngleDeg);
        if (done == 1U)
        {
            ObstacleGuard_SetEnabled(&s_guard, 1U);
            s_route_guard_armed = 1U;
            Avoid_EnterState(AVOID_DIAGONAL);
        }
        else if (done == 2U)
        {
            Avoid_Abort();
        }
        break;

    case AVOID_DIAGONAL:
        if (s_route_guard_armed && obstacle_entered)
        {
            Avoid_Abort();
            break;
        }
        done = Avoid_DistanceDone(Avoid_DiagonalMm, Avoid_ArcSpeed);
        if (done == 1U)
            Avoid_EnterState(AVOID_ARC_LEFT);
        else if (done == 2U)
            Avoid_Abort();
        break;

    case AVOID_ARC_LEFT:
        if (s_route_guard_armed && obstacle_entered)
        {
            Avoid_Abort();
            break;
        }
        done = Avoid_AngleDone(Avoid_LeftAngleDeg);
        if (done == 1U)
            Avoid_EnterState(AVOID_SEARCH_LINE);
        else if (done == 2U)
            Avoid_Abort();
        break;

    case AVOID_SEARCH_LINE:
        if (s_route_guard_armed && obstacle_entered)
        {
            Avoid_Abort();
            break;
        }
        if (Avoid_OnLine())
        {
            if (s_line_confirm < AVOID_LINE_CONFIRM)
                s_line_confirm++;
            if (s_line_confirm >= AVOID_LINE_CONFIRM)
            {
                Avoid_EnterState(AVOID_REENTER_LINE);
                break;
            }
        }
        else
        {
            s_line_confirm = 0;
        }
        if (s_odom_mm >= Avoid_SearchMaxMm)
            Avoid_Abort();
        break;

    case AVOID_REENTER_LINE:
        if (s_route_guard_armed && obstacle_entered)
        {
            Avoid_Abort();
            break;
        }
        if (!Avoid_OnLine())
        {
            Avoid_EnterState(AVOID_SEARCH_LINE);
            break;
        }
        if (Avoid_NearCenter())
        {
            if (s_center_confirm < AVOID_REENTER_CONFIRM)
                s_center_confirm++;
        }
        else
        {
            s_center_confirm = 0;
        }
        if (s_state_ticks >= AVOID_REENTER_MIN_TICKS &&
            s_center_confirm >= AVOID_REENTER_CONFIRM)
        {
            Avoid_Active = 0;
            s_cooldown = AVOID_COOLDOWN_TICKS;
            s_resume_speed = 0;
            ObstacleGuard_SetEnabled(&s_guard, 1U);
            Avoid_EnterState(AVOID_IDLE);
        }
        else if (s_state_ticks >= AVOID_REENTER_TIMEOUT)
        {
            Avoid_Abort();
        }
        break;

    case AVOID_ABORT_HOLD:
        break;

    default:
        Avoid_Abort();
        break;
    }
}

static void Avoid_ComputeOutputs(void)
{
    float turn_scale;

    switch (Avoid_State)
    {
    case AVOID_BRAKE:
    case AVOID_ABORT_HOLD:
        s_avoid_speed = Avoid_Ramp(s_avoid_speed, 0.0f,
                                   AVOID_SPEED_RISE_STEP, Avoid_BrakeFallStep);
        Patrol_Speed_Cmd = s_avoid_speed;
        Patrol_Turn_Cmd = 0;
        break;

    case AVOID_STOP_SETTLE:
        s_avoid_speed = 0;
        Patrol_Speed_Cmd = 0;
        Patrol_Turn_Cmd = 0;
        break;

    case AVOID_ARC_RIGHT:
    case AVOID_ARC_LEFT:
        s_avoid_speed = Avoid_Ramp(s_avoid_speed, Avoid_ArcSpeed,
                                   AVOID_SPEED_RISE_STEP, Avoid_BrakeFallStep);
        Patrol_Speed_Cmd = s_avoid_speed;
        turn_scale = (Avoid_ArcSpeed > 1.0f) ? s_avoid_speed / Avoid_ArcSpeed : 0.0f;
        Patrol_Turn_Cmd = Avoid_ArcTurnTarget * turn_scale;
        if (Avoid_State == AVOID_ARC_LEFT)
            Patrol_Turn_Cmd = -Patrol_Turn_Cmd;
        break;

    case AVOID_DIAGONAL:
        s_avoid_speed = Avoid_Ramp(s_avoid_speed, Avoid_ArcSpeed,
                                   AVOID_SPEED_RISE_STEP, Avoid_BrakeFallStep);
        Patrol_Speed_Cmd = s_avoid_speed;
        Patrol_Turn_Cmd = 0;
        break;

    case AVOID_SEARCH_LINE:
        s_avoid_speed = Avoid_Ramp(s_avoid_speed, Avoid_SearchSpeed,
                                   AVOID_SPEED_RISE_STEP, Avoid_BrakeFallStep);
        Patrol_Speed_Cmd = s_avoid_speed;
        Patrol_Turn_Cmd = 0;
        break;

    case AVOID_REENTER_LINE:
        s_avoid_speed = Avoid_Ramp(s_avoid_speed, Avoid_ReenterSpeed,
                                   AVOID_SPEED_RISE_STEP, Avoid_BrakeFallStep);
        Patrol_Speed_Cmd = s_avoid_speed;
        Patrol_Turn_Cmd = turn_diff * Track_Turn_Scale;
        break;

    default:
        Patrol_Speed_Cmd = 0;
        Patrol_Turn_Cmd = 0;
        break;
    }
}

void TrackAvoid_Supervisor5ms(void)
{
    UsSnapshot snap;
    ObstacleGuardInput in;
    ObstacleGuardOutput out;
    float requested_speed;
    float allowed;
    float target;
    u8 enabled;

    enabled = (Lidar_Detect == 1U && Avoid_Enable) ? 1U : 0U;
    UltrasonicService_Update5ms();
    UltrasonicService_GetSnapshot(&snap);

    if (enabled != s_guard_enabled_prev)
    {
        ObstacleGuard_SetEnabled(&s_guard, enabled);
        s_guard_enabled_prev = enabled;
        if (!enabled && Avoid_Active)
        {
            Avoid_Active = 0;
            s_avoid_speed = 0;
            s_resume_speed = 0;
            s_cooldown = 0;
            Avoid_EnterState(AVOID_IDLE);
        }
    }

    requested_speed = Avoid_Active ? Patrol_Speed_Cmd : base_speed_mm;
    in.requested_speed_mm_s = (int32_t)requested_speed;
    in.measured_forward_speed_mm_s = (int32_t)Avoid_MeasuredSpeed();
    in.distance_mm = snap.distance_mm;
    in.sample_id = snap.sample_id;
    in.sample_valid = snap.valid;
    in.miss_count = snap.miss_count;

    out = ObstacleGuard_Update(&s_guard, &in);
    Guard_State = out.state;
    AvoidRoutine_Update5ms(out.entered_blocked);

    if (Avoid_Active)
    {
        Avoid_ComputeOutputs();
    }
    else
    {
        allowed = (float)out.allowed_speed_mm_s;
        target = (base_speed_mm < allowed) ? base_speed_mm : allowed;
        s_resume_speed = Avoid_Ramp(s_resume_speed, target,
                                    AVOID_SPEED_RISE_STEP, Avoid_BrakeFallStep);
        Patrol_Speed_Cmd = s_resume_speed;
        Patrol_Turn_Cmd = (out.state == OBSTACLE_GUARD_BLOCKED) ?
                          0.0f : turn_diff * Track_Turn_Scale;
    }
}

void TrackAvoid_Init(void)
{
    u8 enabled = (Lidar_Detect == 1U && Avoid_Enable) ? 1U : 0U;

    UltrasonicService_Init();
    ObstacleGuard_Init(&s_guard, enabled);
    s_guard_enabled_prev = enabled;
    s_route_guard_armed = 0;
    s_cooldown = 0;
    s_avoid_speed = 0;
    s_resume_speed = 0;
    Avoid_Active = 0;
    Guard_State = enabled ? OBSTACLE_GUARD_CLEAR : OBSTACLE_GUARD_DISABLED;
    Avoid_EnterState(AVOID_IDLE);
    Patrol_Speed_Cmd = 0;
    Patrol_Turn_Cmd = 0;
}
