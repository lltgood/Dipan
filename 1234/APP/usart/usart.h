#ifndef __usart_H
#define __usart_H

#define Buff_Len 7

#include "system.h" 
#include "stdio.h"

void USART1_Init(u32 bound);
void USART2_Init(u32 bound);
typedef struct  
{
	u8 left_high;
	u8 left_low;
	u8 right_high;
	u8 right_low;
	u8 sign;
}Buff;

#endif
