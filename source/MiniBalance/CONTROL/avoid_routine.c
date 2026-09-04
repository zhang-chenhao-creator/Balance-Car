#include "avoid_routine.h"
#include "obstacle_guard.h"
#include "ultrasonic_service.h"
#include "TrackModule.h"
#include "control.h"
#include "show.h"

extern int   Encoder_Left, Encoder_Right;
extern float Gyro_Turn;
extern u8    Mode;
extern u8    Lidar_Detect;

#define ENC_SUM_TO_MM   (Perimeter / (Encoder_precision * EncoderMultiples * Reduction_Ratio * 2.0f))
#define GYRO_Z_TO_DPS   (1.0f / 16.4f)
#define DT_S            0.005f

float Patrol_Speed_Cmd = 0;
float Patrol_Turn_Cmd  = 0;
u8 Avoid_Active = 0;
u8 Avoid_State  = AVOID_IDLE;
u8 Avoid_Enable = 1;
u8 Guard_State  = 0;

float Avoid_D1_mm      = 250.0f;
float Avoid_D2_mm      = 400.0f;
float Avoid_SeekMax_mm = 800.0f;
float Avoid_LegSpeed   = 120.0f;
float Avoid_SeekSpeed  = 100.0f;
float Avoid_TurnTarget = 27.0f;
float Avoid_TurnAngle  = 90.0f;

static ObstacleGuardContext s_guard;
static u8      s_guard_enabled_prev = 0xFF;
static float   s_odom_mm;
static float   s_turn_deg;
static u16     s_state_ticks;
static u16     s_block_confirm;
static u8      s_center_ok;
static u16     s_cooldown;
static float   s_resume_speed;

#define AVOID_BLOCK_CONFIRM      3U
#define AVOID_STOP_SETTLE_TICKS  80U    /* 400ms */
#define AVOID_TURN_TIMEOUT       600U   /* 3s */
#define AVOID_ALIGN_MIN_DEG      60.0f  /* 对准至少转过这么多才允许用灰度提前结束 */
#define AVOID_REJOIN_TICKS       120U   /* 600ms 低速巡线再交还 */
#define AVOID_REJOIN_CENTER      6U
#define AVOID_COOLDOWN_TICKS     400U   /* 2s */

static void Avoid_EnterState(u8 next)
{
    Avoid_State   = next;
    s_odom_mm     = 0;
    s_turn_deg    = 0;
    s_state_ticks = 0;
    s_center_ok   = 0;
}

static u8 Avoid_LegDone(float target_mm, float speed_mm_s)
{
    u32 timeout_ticks = (u32)(target_mm / speed_mm_s * 1000.0f / 5.0f * 2.5f) + 200U;
    if (s_odom_mm >= target_mm) return 1;
    if (s_state_ticks >= timeout_ticks) return 2;
    return 0;
}

static u8 Avoid_TurnDone(void)
{
    if (s_turn_deg >= Avoid_TurnAngle) return 1;
    if (s_state_ticks >= AVOID_TURN_TIMEOUT) return 2;
    return 0;
}

static u8 Avoid_OnLine(void)
{
    return Track_state != STATE_LOST;
}

static u8 Avoid_NearCenter(void)
{
    return (Track_state == STATE_STRAIGHT) ||
           (Track_state == STATE_LEFT_SMALL) ||
           (Track_state == STATE_RIGHT_SMALL);
}

static void AvoidRoutine_Update5ms(u8 guard_blocked)
{
    s_odom_mm  += (Encoder_Left + Encoder_Right) * ENC_SUM_TO_MM;
    s_turn_deg += (Gyro_Turn >= 0 ? Gyro_Turn : -Gyro_Turn) * GYRO_Z_TO_DPS * DT_S;
    s_state_ticks++;

    switch (Avoid_State)
    {
    case AVOID_IDLE:
        Avoid_Active = 0;
        if (s_cooldown) s_cooldown--;
        if (guard_blocked)
        {
            if (s_block_confirm < 255U) s_block_confirm++;
            if (s_block_confirm >= AVOID_BLOCK_CONFIRM && Avoid_Enable && s_cooldown == 0)
            {
                Avoid_Active = 1;
                Avoid_EnterState(AVOID_STOP_SETTLE);
            }
        }
        else
        {
            s_block_confirm = 0;
        }
        break;

    case AVOID_STOP_SETTLE:
        if (s_state_ticks >= AVOID_STOP_SETTLE_TICKS)
            Avoid_EnterState(AVOID_TURN1_RIGHT);
        break;

    case AVOID_TURN1_RIGHT:
        {
            u8 done = Avoid_TurnDone();
            if (done == 1) Avoid_EnterState(AVOID_LEG1);
            else if (done == 2) Avoid_EnterState(AVOID_ABORT_HOLD);
        }
        break;

    case AVOID_LEG1:
        if (guard_blocked) { Avoid_EnterState(AVOID_ABORT_HOLD); break; }
        {
            u8 done = Avoid_LegDone(Avoid_D1_mm, Avoid_LegSpeed);
            if (done == 1) Avoid_EnterState(AVOID_TURN2_LEFT);
            else if (done == 2) Avoid_EnterState(AVOID_ABORT_HOLD);
        }
        break;

    case AVOID_TURN2_LEFT:
        {
            u8 done = Avoid_TurnDone();
            if (done == 1) Avoid_EnterState(AVOID_LEG2);
            else if (done == 2) Avoid_EnterState(AVOID_ABORT_HOLD);
        }
        break;

    case AVOID_LEG2:
        if (guard_blocked) { Avoid_EnterState(AVOID_ABORT_HOLD); break; }
        {
            u8 done = Avoid_LegDone(Avoid_D2_mm, Avoid_LegSpeed);
            if (done == 1) Avoid_EnterState(AVOID_TURN3_LEFT);
            else if (done == 2) Avoid_EnterState(AVOID_ABORT_HOLD);
        }
        break;

    case AVOID_TURN3_LEFT:
        {
            u8 done = Avoid_TurnDone();
            if (done == 1) Avoid_EnterState(AVOID_SEEK_LINE);
            else if (done == 2) Avoid_EnterState(AVOID_ABORT_HOLD);
        }
        break;

    case AVOID_SEEK_LINE:
        if (guard_blocked) { Avoid_EnterState(AVOID_ABORT_HOLD); break; }
        if (Avoid_OnLine())
        {
            Avoid_EnterState(AVOID_ALIGN_RIGHT);
            break;
        }
        if (s_odom_mm >= Avoid_SeekMax_mm)
            Avoid_EnterState(AVOID_ABORT_HOLD);
        break;

    case AVOID_ALIGN_RIGHT:
        /* 垂直压线时长时间是 0000，不能只等 1001。
         * 主条件：陀螺仪转到约 90°（与前三段原地转一致）。
         * 转过 60° 后若已经居中/微调，可以提前结束，避免转过头。 */
        if (s_turn_deg >= Avoid_TurnAngle)
        {
            Avoid_EnterState(AVOID_REJOIN_FOLLOW);
            break;
        }
        if (s_turn_deg >= AVOID_ALIGN_MIN_DEG && Avoid_NearCenter())
        {
            if (s_center_ok < 255U) s_center_ok++;
            if (s_center_ok >= 4U)
            {
                Avoid_EnterState(AVOID_REJOIN_FOLLOW);
                break;
            }
        }
        else
        {
            s_center_ok = 0;
        }
        if (s_state_ticks >= AVOID_TURN_TIMEOUT)
            Avoid_EnterState(AVOID_REJOIN_FOLLOW);
        break;

    case AVOID_REJOIN_FOLLOW:
        /* 低速把 e1c316e 巡线接回来，不再原地停 2.5s。 */
        if (Track_state == STATE_STRAIGHT)
        {
            if (s_center_ok < 255U) s_center_ok++;
        }
        else
        {
            s_center_ok = 0;
        }
        if ((s_state_ticks >= 40U && s_center_ok >= AVOID_REJOIN_CENTER) ||
            (s_state_ticks >= AVOID_REJOIN_TICKS))
        {
            Avoid_Active = 0;
            s_block_confirm = 0;
            s_cooldown = AVOID_COOLDOWN_TICKS;
            s_resume_speed = 0;
            Avoid_EnterState(AVOID_IDLE);
        }
        break;

    case AVOID_ABORT_HOLD:
        if (Avoid_OnLine())
            Avoid_EnterState(AVOID_ALIGN_RIGHT);
        break;

    default:
        Avoid_EnterState(AVOID_IDLE);
        break;
    }
}

static void Avoid_ComputeOutputs(void)
{
    switch (Avoid_State)
    {
    case AVOID_STOP_SETTLE:
    case AVOID_ABORT_HOLD:
        Patrol_Speed_Cmd = 0;
        Patrol_Turn_Cmd  = 0;
        break;
    case AVOID_TURN1_RIGHT:
    case AVOID_ALIGN_RIGHT:
        Patrol_Speed_Cmd = 0;
        Patrol_Turn_Cmd  = Avoid_TurnTarget;
        break;
    case AVOID_TURN2_LEFT:
    case AVOID_TURN3_LEFT:
        Patrol_Speed_Cmd = 0;
        Patrol_Turn_Cmd  = -Avoid_TurnTarget;
        break;
    case AVOID_LEG1:
    case AVOID_LEG2:
        Patrol_Speed_Cmd = Avoid_LegSpeed;
        Patrol_Turn_Cmd  = 0;
        break;
    case AVOID_SEEK_LINE:
        Patrol_Speed_Cmd = Avoid_SeekSpeed;
        Patrol_Turn_Cmd  = 0;
        break;
    case AVOID_REJOIN_FOLLOW:
        Patrol_Speed_Cmd = LostSpeed;
        Patrol_Turn_Cmd  = turn_diff * Track_Turn_Scale;
        break;
    default:
        Patrol_Speed_Cmd = 0;
        Patrol_Turn_Cmd  = 0;
        break;
    }
}

void TrackAvoid_Supervisor5ms(void)
{
    UsSnapshot snap;
    ObstacleGuardInput  in;
    ObstacleGuardOutput out;
    u8 enabled = (Lidar_Detect == 1) ? 1U : 0U;

    UltrasonicService_Update5ms();
    UltrasonicService_GetSnapshot(&snap);

    if (enabled != s_guard_enabled_prev)
    {
        ObstacleGuard_SetEnabled(&s_guard, enabled);
        s_guard_enabled_prev = enabled;
        if (!enabled && Avoid_Active)
        {
            Avoid_Active = 0;
            s_resume_speed = 0;
            s_cooldown = 0;
            Avoid_EnterState(AVOID_IDLE);
        }
    }

    in.requested_speed_mm_s         = (int32_t)base_speed_mm;
    in.measured_forward_speed_mm_s  = (int32_t)((Velocity_Left + Velocity_Right) / 2.0f);
    in.distance_mm                  = snap.distance_mm;
    in.sample_id                    = snap.sample_id;
    in.sample_valid                 = snap.valid;
    in.miss_count                   = snap.miss_count;

    out = ObstacleGuard_Update(&s_guard, &in);
    Guard_State = out.state;

    AvoidRoutine_Update5ms((out.state == OBSTACLE_GUARD_BLOCKED) ? 1U : 0U);

    if (Avoid_Active)
    {
        Avoid_ComputeOutputs();
    }
    else
    {
        float allowed = (float)out.allowed_speed_mm_s;
        float target = (base_speed_mm < allowed) ? base_speed_mm : allowed;
        if (s_resume_speed < target)
        {
            s_resume_speed += 5.0f;
            if (s_resume_speed > target) s_resume_speed = target;
        }
        else
        {
            s_resume_speed = target;
        }
        Patrol_Speed_Cmd = s_resume_speed;
        if (out.state == OBSTACLE_GUARD_BLOCKED)
            Patrol_Turn_Cmd = 0;
        else
            Patrol_Turn_Cmd = turn_diff * Track_Turn_Scale;
    }
}

void TrackAvoid_Init(void)
{
    UltrasonicService_Init();
    ObstacleGuard_Init(&s_guard, (Lidar_Detect == 1) ? 1U : 0U);
    s_guard_enabled_prev = (Lidar_Detect == 1) ? 1U : 0U;
    Guard_State = OBSTACLE_GUARD_CLEAR;
    Avoid_Active = 0;
    s_block_confirm = 0;
    s_cooldown = 0;
    s_resume_speed = 0;
    Avoid_EnterState(AVOID_IDLE);
    Patrol_Speed_Cmd = 0;
    Patrol_Turn_Cmd  = 0;
}
