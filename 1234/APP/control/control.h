#ifndef _control_H
#define _control_H

#include "system.h"
#include "CAN_Receive.h"
#include "pid.h"

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//底盘电机速度环PID
#define M3505_MOTOR1_SPEED_PID_KP 2.05f
#define M3505_MOTOR1_SPEED_PID_KI 0.00f
#define M3505_MOTOR1_SPEED_PID_KD 0.38f

#define M3505_MOTOR2_SPEED_PID_KP 2.25f
#define M3505_MOTOR2_SPEED_PID_KI 0.00f
#define M3505_MOTOR2_SPEED_PID_KD 0.30f

#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;		//motor_measure_t 定义在"CAN_Receive.h"

typedef struct
{
  Chassis_Motor_t motor_chassis[2];          //底盘电机数据
  PidTypeDef motor_speed_pid[2];             //底盘电机速度pid
} chassis_move_t;

/*增量式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 
 float errNow; //当前的误差
 float dCtrOut;//控制增量输出
 float ctrOut;//控制输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld1;
 float errOld2;
 
}PID_IncrementType;

//底盘初始化，主要是pid初始化
 void chassis_init(chassis_move_t *chassis_move_init);
//PID变量清零函数
 void PID_Move_Clear(chassis_move_t *chassis_move_clear);
//底盘初始化，主要是pid初始化 
 void chassis_init(chassis_move_t *chassis_move_init);
//底盘数据更新
 void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘PID计算以及运动分解
 void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//2闭环
 void Motor_error_equalize(chassis_move_t *motor_error_e);
 void PID_IncrementMode(PID_IncrementType* PID);
#endif
