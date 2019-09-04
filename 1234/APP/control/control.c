#include "stm32f10x.h"
#include "can.h"
#include "led.h"
#include "CAN_Receive.h"
#include "math.h"
#include "stdlib.h"
#include "control.h"
#include "time.h"

u8 flag[2]={0,0};
s32 spdTag, spdNow, control;//����һ��Ŀ���ٶȣ������ٶȣ�������
int32_t speed1,speed2;
int32_t wheel_speed[2] = {0, 0};
chassis_move_t chassis_move;

PID_IncrementType PID_Control;//����PID�㷨�Ľṹ��

//PID������ʼ��
void PID_Move_Clear(chassis_move_t *chassis_move_clear)
{
	u8 i=0;
	for(i=0;i<2;i++)
		PID_clear(&chassis_move_clear->motor_speed_pid[i]);
}

void chassis_init(chassis_move_t *chassis_move_init)
{
	uint8_t i;
	//�����ٶȻ�pidֵ
	const static fp32 motor1_speed_pid[3] = {M3505_MOTOR1_SPEED_PID_KP, M3505_MOTOR1_SPEED_PID_KI, M3505_MOTOR1_SPEED_PID_KD};
	const static fp32 motor2_speed_pid[3] = {M3505_MOTOR2_SPEED_PID_KP, M3505_MOTOR2_SPEED_PID_KI, M3505_MOTOR2_SPEED_PID_KD};
	if (chassis_move_init == NULL)
	{
			return;
	}
	//��ʼ��PID 
	for (i = 0; i < 2; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);		
		PID_clear(&chassis_move_init->motor_speed_pid[i]);
	}
	PID_Init(&chassis_move_init->motor_speed_pid[0], PID_DELTA, motor1_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	PID_Init(&chassis_move_init->motor_speed_pid[1], PID_DELTA, motor2_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	//����һ������
	chassis_feedback_update(chassis_move_init);
}
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	uint8_t i = 0;
	if (chassis_move_update == NULL)
	{
		return;
	}
	for (i = 0; i < 2; i++)
	{
			//���µ���ٶȣ����ٶ����ٶȵ�PID΢��
		chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0];
	}
}

void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	uint8_t i = 0;
/*************************************Ŀ���ٶ�******************************************/
//	 wheel_speed[0]=0;
//	 wheel_speed[1]=0;
//	printf(" %5d %5d\r\n",wheel_speed[0],wheel_speed[1]);
/**************************************************************************************/		
	//�������ӿ�������ٶȣ�������������ٶ�
	for (i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
	}
	/*****************��ת��ʱ��״̬�ж�********************/
		if(abs(chassis_move_control_loop->motor_chassis[0].speed)>abs(chassis_move_control_loop->motor_chassis[0].speed_set)*0.6||
			abs(chassis_move_control_loop->motor_chassis[0].speed)<abs(chassis_move_control_loop->motor_chassis[0].speed_set)*1.4)
		{
			TIM_Cmd(TIM2,DISABLE); //�رն�ʱ��2	
			flag[0]=0;
		}
		else
			TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
		if(abs(chassis_move_control_loop->motor_chassis[1].speed)>abs(chassis_move_control_loop->motor_chassis[1].speed_set)*0.6||
			abs(chassis_move_control_loop->motor_chassis[1].speed)<abs(chassis_move_control_loop->motor_chassis[1].speed_set)*1.4)
		{
			TIM_Cmd(TIM3,DISABLE); //�رն�ʱ��3	
			flag[1]=0;
		}
		else
			TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	//����pid
	for (i = 0; i < 2; i++)
	{
		PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set,flag[i]);
	}
	//��ֵ����ֵ
	for (i = 0; i < 2; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
	}
}

void Motor_error_equalize(chassis_move_t *motor_error_e)
{
	if(abs(motor_error_e->motor_chassis[0].speed)>abs(motor_error_e->motor_chassis[1].speed))
	{
		spdTag=motor_error_e->motor_chassis[0].speed;
		spdNow=motor_error_e->motor_chassis[1].speed;
	}
	else if(abs(motor_error_e->motor_chassis[0].speed)<abs(motor_error_e->motor_chassis[1].speed))
	{
		spdTag=motor_error_e->motor_chassis[1].speed;
		spdNow=motor_error_e->motor_chassis[0].speed;
	}
	 PID_Control.errNow = spdTag + spdNow; //���㲢д���ٶ����
		
	 PID_Control.kp      = 2.5;             //д�����ϵ��Ϊ15
	 PID_Control.ki      = 0.2;              //д�����ϵ��Ϊ5
	 PID_Control.kd      = 1.5;              //д��΢��ϵ��Ϊ5

	 PID_IncrementMode(&PID_Control);       //ִ�о���ʽPID�㷨
	
	 control = PID_Control.ctrOut;         //��ȡ����ֵ

}

void PID_IncrementMode(PID_IncrementType* PID)
{
	 float dErrP, dErrI, dErrD;
	 
	 if(PID->kp < 0)    PID->kp = -PID->kp;
	 if(PID->ki < 0)	PID->ki = -PID->ki;
	 if(PID->kd < 0)    PID->kd = -PID->kd;

	 dErrP = PID->errNow - PID->errOld1;

	 dErrI = PID->errNow;

	 dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2;

	 PID->errOld2 = PID->errOld1; //�������΢��
	 PID->errOld1 = PID->errNow;  //һ�����΢��

	 /*����ʽPID����*/
	 PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD;
	 
	 if(PID->kp == 0 && PID->ki == 0 && PID->kd == 0)   PID->ctrOut = 0;

	 else PID->ctrOut += PID->dCtrOut;
}
