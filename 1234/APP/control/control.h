#ifndef _control_H
#define _control_H

#include "system.h"
#include "CAN_Receive.h"
#include "pid.h"

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//���̵���ٶȻ�PID
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
} Chassis_Motor_t;		//motor_measure_t ������"CAN_Receive.h"

typedef struct
{
  Chassis_Motor_t motor_chassis[2];          //���̵������
  PidTypeDef motor_speed_pid[2];             //���̵���ٶ�pid
} chassis_move_t;

/*����ʽPID�㷨���ӿڲ����ṹ����*/
typedef struct 
{
 /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
 float kp;     //����ϵ��
 float ki;     //����ϵ��
 float kd;     //΢��ϵ��
 
 float errNow; //��ǰ�����
 float dCtrOut;//�����������
 float ctrOut;//�������
 
 /*PID�㷨�ڲ���������ֵ�����޸�*/
 float errOld1;
 float errOld2;
 
}PID_IncrementType;

//���̳�ʼ������Ҫ��pid��ʼ��
 void chassis_init(chassis_move_t *chassis_move_init);
//PID�������㺯��
 void PID_Move_Clear(chassis_move_t *chassis_move_clear);
//���̳�ʼ������Ҫ��pid��ʼ�� 
 void chassis_init(chassis_move_t *chassis_move_init);
//�������ݸ���
 void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����PID�����Լ��˶��ֽ�
 void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//2�ջ�
 void Motor_error_equalize(chassis_move_t *motor_error_e);
 void PID_IncrementMode(PID_IncrementType* PID);
#endif
