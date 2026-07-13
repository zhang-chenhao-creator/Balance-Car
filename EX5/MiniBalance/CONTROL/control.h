#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265
#define Control_Frequency 200.0
#define Diameter_67 67.0
#define EncoderMultiples 4.0
#define Encoder_precision 500.0
#define Reduction_Ratio 30.0
#define Perimeter 210.4867

#define Normal_Mode 0

/* Safety_Stop_Reason 编码，用于 STATUS 判断为什么 PWM 没有真正输出。 */
#define SAFE_RUN        0
#define SAFE_FALL       1
#define SAFE_KEY        2
#define SAFE_SOFT_STOP  3
#define SAFE_LOW_VOLT   4
#define SAFE_PICK_UP    5

/* Ultrasonic safety supervisor states for STATUS/OLED display. */
#define ULTRA_STATE_DISABLED 0
#define ULTRA_STATE_CLEAR    1
#define ULTRA_STATE_SLOW     2
#define ULTRA_STATE_BLOCKED  3
#define ULTRA_STATE_DEGRADED 4

/*
 * 控制层函数索引：
 * EXTI9_5_IRQHandler：MPU6050 数据中断，也是平衡闭环主循环。
 * Balance：角度环 PD，负责车身不倒。
 * Velocity：速度环 PI，负责抑制前后漂移。
 * Set_Pwm：把控制量写到 TIM3 四路 PWM。
 * Turn_Off：统一安全停机判断。
 * Get_Angle：从 MPU6050/DMP/Kalman/互补滤波得到姿态角。
 */
int EXTI9_5_IRQHandler(void);
int Balance(float angle, float gyro);
int Velocity(int encoder_left, int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left, int motor_right);
int PWM_Limit(int IN, int max, int min);
u8 Turn_Off(float angle, int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right);
int Put_Down(float Angle, int encoder_left, int encoder_right);
void Get_Velocity_Form_Encoder(int encoder_left, int encoder_right);
void Ultrasonic_Guard_Init(void);
void Ultrasonic_Guard_SetEnable(u8 enable);

extern short Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
extern float Velocity_Left, Velocity_Right;
extern volatile int Ultrasonic_Request_Speed, Ultrasonic_Safe_Speed;
extern volatile u16 Ultrasonic_Stop_Distance, Ultrasonic_Slow_Distance;

#endif

