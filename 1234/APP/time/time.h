#ifndef _time_H
#define _time_H

#include "system.h"
#include "CAN_Receive.h"
#include "pid.h"

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
//#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
//#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�����������Ƶ�ʣ���δʹ�������
//#define CHASSIS_CONTROL_FREQUENCE 500.0f
//���̵������ٶ�
//#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
//#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f

void TIM2_Init(u16 per,u16 psc);
void TIM3_Init(u16 per,u16 psc);
void TIM4_Init(u16 per,u16 psc);
#endif


