#include "hard.h"

#include "main.h"

void hardware_init() {
    delay_init(100);  //时钟初始化

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断分组配置

    NVIC_SetPriority(SysTick_IRQn, 0);  //设置滴答定时器中断优先级最高

    // led_init();
    TIM3_Cap_Init(100);

    TIM1_Pwm_Init(2500, 100 - 1);  // 400Hz

    Usart2_init(460800);

    Usart2_DMA_Config();

    Ultrasonic_Init();

    MPU_Init();  // MPU 初始化

    HMCInit();  // HMC 初始化

    Control_Para_Init();
}
