#ifndef __USART2_H
#define __USART2_H

#include "stm32f4xx.h"

void Usart2_init(u32 bound);
void Usart2_DMA_Config(void);
void DMA_BUF_Count_Init(void);
void Usart2_DMA_Send_Start(void);
	
#endif
