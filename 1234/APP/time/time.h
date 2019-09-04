#ifndef _time_H
#define _time_H

#include "system.h"
#include "CAN_Receive.h"
#include "pid.h"

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘任务控制频率，尚未使用这个宏
//#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘电机最大速度
//#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
//#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f

void TIM2_Init(u16 per,u16 psc);
void TIM3_Init(u16 per,u16 psc);
void TIM4_Init(u16 per,u16 psc);
#endif


