#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f4xx.h"

#define NVIC_TIM4_ULTAR_P 2
#define NVIC_TIM4_ULTAR_S 1
#define TIM4_PERIOD_VALUE 0xffff

void Ultrasonic_Init(void);
void Ultra_Start(void);

extern s8 ultra_start_f;
extern u16 ultra_get_data;

#endif
