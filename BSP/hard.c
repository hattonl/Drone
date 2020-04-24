#include "hard.h"

#include "main.h"

void hardware_init() {
    delay_init(100);  //ʱ�ӳ�ʼ��

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�жϷ�������

    NVIC_SetPriority(SysTick_IRQn, 0);  //���õδ�ʱ���ж����ȼ����

    // led_init();
    TIM3_Cap_Init(100);

    TIM1_Pwm_Init(2500, 100 - 1);  // 400Hz

    Usart2_init(460800);

    Usart2_DMA_Config();

    Ultrasonic_Init();

    MPU_Init();  // MPU ��ʼ��

    HMCInit();  // HMC ��ʼ��

    Control_Para_Init();
}
