#include "sys.h"

/*
 * OLED status pages for EX4.
 *
 * The OLED is not part of the balance control loop. It only mirrors runtime
 * state so the demonstration can show what the car is doing:
 *   page 1: Bluetooth command, arm/stop state, angle, battery, ultrasonic distance
 *   page 2: PID parameters, middle angle, left/right measured velocity
 */
#define OLED_REFRESH_TICKS 3
#define OLED_PAGE_TICKS    46

static u8 status_page;
static u8 refresh_ticks;
static u8 page_ticks;

static const u8 *Bluetooth_Command_Text(void)
{
    /* Four-character labels fit the 0.96-inch OLED layout. */
    switch (Bluetooth_Command)
    {
        case BT_CMD_FORWARD:  return (const u8 *)"FWD ";
        case BT_CMD_BACKWARD: return (const u8 *)"BACK";
        case BT_CMD_LEFT:     return (const u8 *)"LEFT";
        case BT_CMD_RIGHT:    return (const u8 *)"RGHT";
        case BT_CMD_ULTRASONIC_AVOID: return (const u8 *)"AUTO";
        default:              return (const u8 *)"STOP";
    }
}

static const u8 *Ultrasonic_Action_Text(void)
{
    switch (Ultrasonic_Avoid_Action)
    {
        case ULTRA_STATE_DISABLED: return (const u8 *)"OFF";
        case ULTRA_STATE_SLOW:     return (const u8 *)"SLW";
        case ULTRA_STATE_BLOCKED:  return (const u8 *)"BLK";
        case ULTRA_STATE_DEGRADED: return (const u8 *)"DGD";
        default:                   return (const u8 *)"CLR";
    }
}

static u32 Signed_Magnitude(int value)
{
    if (value < 0)
        return (u32)(-value);
    return (u32)value;
}

static void OLED_Show_Signed(u8 x, u8 y, int value, u8 digits)
{
    OLED_ShowChar(x, y, (value < 0) ? '-' : '+', 12, 1);
    OLED_ShowNumber(x + 8, y, Signed_Magnitude(value), digits, 12);
}

static void OLED_Clear_Buffer(void)
{
    u8 page;
    u8 x;

    for (page = 0; page < 8; page++)
    {
        for (x = 0; x < 128; x++)
            OLED_GRAM[x][page] = 0;
    }
}

static void OLED_Draw_Remote_Page(void)
{
    OLED_ShowString(0, 0, (const u8 *)"EX4 BT      1/2");

    OLED_ShowString(0, 12, (const u8 *)"CMD:");
    OLED_ShowString(32, 12, Bluetooth_Command_Text());
    if (Flag_Stop == 0 && KEY2_STATE == 0)
        OLED_ShowString(88, 12, (const u8 *)"ARM ");
    else
        OLED_ShowString(88, 12, (const u8 *)"STOP");

    OLED_ShowString(0, 24, (const u8 *)"ANG:");
    OLED_Show_Signed(32, 24, (int)Angle_Balance, 3);

    OLED_ShowString(0, 36, (const u8 *)"BAT:");
    OLED_ShowNumber(32, 36, (u32)(Voltage / 100), 2, 12);
    OLED_ShowString(48, 36, (const u8 *)".");
    OLED_ShowNumber(56, 36, (u32)((Voltage / 10) % 10), 1, 12);
    OLED_ShowString(64, 36, (const u8 *)"V");

    OLED_ShowString(0, 48, (const u8 *)"DIS:");
    if (Ultrasonic_Valid)
    {
        OLED_ShowNumber(32, 48, Distance, 4, 12);
        OLED_ShowString(72, 48, (const u8 *)"mm");
    }
    else
    {
        OLED_ShowString(32, 48, (const u8 *)"----");
        OLED_ShowString(72, 48, (const u8 *)"mm");
    }
    OLED_ShowString(96, 48, Ultrasonic_Action_Text());
}

static void OLED_Draw_Control_Page(void)
{
    OLED_ShowString(0, 0, (const u8 *)"CONTROL     2/2");

    OLED_ShowString(0, 12, (const u8 *)"BK:");
    OLED_ShowNumber(24, 12, (u32)Balance_Kp, 5, 12);
    OLED_ShowString(72, 12, (const u8 *)"BD:");
    OLED_ShowNumber(96, 12, (u32)Balance_Kd, 3, 12);

    OLED_ShowString(0, 24, (const u8 *)"VK:");
    OLED_ShowNumber(24, 24, (u32)Velocity_Kp, 3, 12);
    OLED_ShowString(64, 24, (const u8 *)"VI:");
    OLED_ShowNumber(88, 24, (u32)Velocity_Ki, 3, 12);

    OLED_ShowString(0, 36, (const u8 *)"MID:");
    OLED_Show_Signed(32, 36, Middle_angle, 3);

    OLED_ShowString(0, 48, (const u8 *)"VL:");
    OLED_Show_Signed(24, 48, (int)Velocity_Left, 3);
    OLED_ShowString(64, 48, (const u8 *)"VR:");
    OLED_Show_Signed(88, 48, (int)Velocity_Right, 3);
}

static void OLED_Status_Render(void)
{
    /* Render into the local OLED buffer first, then refresh the screen once. */
    OLED_Clear_Buffer();
    if (status_page == 0)
        OLED_Draw_Remote_Page();
    else
        OLED_Draw_Control_Page();
    OLED_Refresh_Gram();
}

void OLED_Status_Init(void)
{
    status_page = 0;
    refresh_ticks = 0;
    page_ticks = 0;
    OLED_Init();
    OLED_Status_Render();
}

void OLED_Status_Update_65ms(void)
{
    /*
     * Called from the main loop at roughly 65 ms intervals.
     * The screen refreshes every 195 ms and switches page about every 3 seconds.
     */
    refresh_ticks++;
    page_ticks++;

    if (page_ticks >= OLED_PAGE_TICKS)
    {
        page_ticks = 0;
        status_page = !status_page;
        refresh_ticks = OLED_REFRESH_TICKS;
    }

    if (refresh_ticks >= OLED_REFRESH_TICKS)
    {
        refresh_ticks = 0;
        OLED_Status_Render();
    }
}
