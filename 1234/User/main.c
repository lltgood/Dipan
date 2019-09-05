/**********************************************************
*Project Name:chassis motor control(Demon)
*Task:				M3508 control
*							PID	calculate
*							PS2 control
*Author:			Huang.F.Y.
*Update:			PS2 debug	
*Date:				2019.8.15
************************************************************
***************************管脚图***************************
											STM32F103C8T6
PA9	->USART1 RX					PC16->led3(time3)
PA10->USART1 TX					PB12->PS2 DI(DAT)
PA11->CAN1 RX						PB13->PS2 DO(CMD)
PA12->CAN1 TX						PB14->PS2 CS
PC14->led1(main)				PB15->PS2 CLK
PC15->led2(time4)
************************************************************/

#include "SysTick.h"
#include "system.h"
#include "led.h"
#include "pid.h"
#include "can.h"
#include "usart.h"
#include "time.h"
#include "oled.h"
#include "main.h"
#include "stdlib.h"
#include "ps2.h"
#include "control.h"


void All_Init(void);

extern Buff speed_set_buff;
extern int16_t wheel_speed[2];
//底盘运动数据
extern chassis_move_t chassis_move;
extern u8 usart1_buff[Buff_Len];
int main()
{
	u8 key=0;
	s16 speed; 
	s16 swerve;
	u8 i=0; 
	All_Init();
	
	while(1)
	{		
		led2=!led2;
		key=PS2_DataKey();
		if(speed_set_buff.sign=='Y')
		{
				if(key!=0)                   //有按键按下
				{
						if(key == 11)
						{
								speed = PS2_AnologData(PSS_LY)-127;
								swerve = (PS2_AnologData(PSS_RX)-128)*12*((float)abs(speed)/128); //	speed取绝对值，	算数运算，得到转弯量。
								speed = -(PS2_AnologData(PSS_LY)-127)*24;	   //正：前进；  负：后退
								if(speed==0&&swerve==0)
								{
									wheel_speed[0]=0;
									wheel_speed[1]=0;
									PID_Move_Clear(&chassis_move);
								}
								else
								{
										if(speed > 0) //向前
										{
												if(swerve < 0)//左转弯
												{
													wheel_speed[0]=(speed + swerve);
													wheel_speed[1]=(~speed+1);
												}
												else          //右转弯
												{
													wheel_speed[0]=(speed);
													wheel_speed[1]=(~(speed - swerve)+1);
												}
										}
										else if(speed < 0)//向后
										{
												if(swerve < 0)//左转弯
												{	
													wheel_speed[0]=(speed - swerve);
													wheel_speed[1]=(~speed+1);
												}
												else//右转弯
												{
													wheel_speed[0]=(speed);
													wheel_speed[1]=(~(speed + swerve)+1);
												}
										}
								}
						}
						else 
						{
								wheel_speed[0]=0;
								wheel_speed[1]=0;
								PID_Move_Clear(&chassis_move);
						}
						if(key == 12)
						{
							PS2_Vibration(0x00,0xFF);  //发出震动后必须有延时  delay_ms(1000);
							delay_ms(500); 
						}
						else
						 PS2_Vibration(0x00,0x00); 
				}
				else
				{
					wheel_speed[0]=0;
					wheel_speed[1]=0;
					PID_Move_Clear(&chassis_move);
				}
				delay_ms(15);		//延时等待，为了保持手柄连接良好
		}
		if(speed_set_buff.sign=='N')
		{
			wheel_speed[0]=(int16_t)(speed_set_buff.left_high <<8 |speed_set_buff.left_low );
			wheel_speed[1]=(int16_t)(speed_set_buff.right_high<<8|speed_set_buff.right_low);
			wheel_speed[1]=-wheel_speed[1];
			wheel_speed[0]= 1000;
			wheel_speed[1]= -3000;
		}
		
		chassis_feedback_update(&chassis_move);
		Motor_error_equalize(&chassis_move);
		//底盘控制PID计算
		chassis_control_loop(&chassis_move);
		
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current);
		i++;
		if(i%20==0)
		{
			led1=!led1;
		}
		}
}

void All_Init()
{
	SysTick_Init(72);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);  //中断优先级分组 分3组
	OLED_Init();			//初始化OLED 
	LED_Init();
	PS2_Init();					//PS2管脚初始化
	PS2_SetInit();		 //PS2配置初始化,配置“红绿灯模式”，并选择是否可以修改
	                   //开启震动模式
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,2,CAN_Mode_Normal);	//1Mbps波特率
	USART1_Init(9600);		//串口1初始化
	USART2_Init(9600);
	chassis_init(&chassis_move);//底盘初始化
	delay_ms(50);
	TIM2_Init(3000,36000-1);//1.5秒定时
	TIM3_Init(3000,36000-1);//1.5秒定时
//	TIM4_Init(2000,72-1);  //定时4ms
	delay_ms(20);		//等待初始化
}
