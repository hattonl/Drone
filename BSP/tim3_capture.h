#ifndef __TIM3_CAPTURE
#define __TIM3_CAPTURE

#include "includes.h"
#include "stm32f4xx.h"
/*
enum
{
 ROL= 0,
 PIT ,
 THR ,
 YAW ,
 CH_NUM
};*/

enum { YAW = 0, THR, PIT, ROL, CH_NUM };

void TIM3_Cap_Init(u16 psc);
// extern u16 Tim3_Pwm_Capt[CH_NUM] = {1500,1000,1500,1500};

#endif
