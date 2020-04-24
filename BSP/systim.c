#include "systim.h"
// use only os
#include "includes.h"

static u8 fac_us = 0;   // us��ʱ������
static u16 fac_ms = 0;  // ms��ʱ������,��os��,����ÿ�����ĵ�ms��

extern void PutTaskRun(void);

void SysTick_Handler(void) {
    if (OSRunning == 1)  // OS��ʼ����,��ִ�������ĵ��ȴ���
    {
        OSIntEnter();  //�����ж�
        PutTaskRun();
        OSTimeTick();  //����ucos��ʱ�ӷ������
        OSIntExit();  //���������л����ж�
    }
}

void delay_init(u8 SYSCLK) {
    u32 reload;

    SysTick_CLKSourceConfig(
        SysTick_CLKSource_HCLK_Div8);  // SYSTICKʹ���ⲿʱ��Դ
    fac_us = SYSCLK / 8;  //�����Ƿ�ʹ��OS,fac_us����Ҫʹ��

    reload = SYSCLK / 8;  //ÿ���ӵļ������� ��λΪK
    reload *= 1000000 / OSCfg_TickRate_Hz;
			//����delay_ostickspersec�趨���ʱ��
            // reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����
    fac_ms = 1000 / OSCfg_TickRate_Hz;  //����OS������ʱ�����ٵ�λ
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  //����SYSTICK�ж�
    SysTick->LOAD = reload;  //ÿ1/OS_TICKS_PER_SEC���ж�һ��
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //����SYSTICK

    fac_ms = (u16)fac_us * 1000;  //��OS��,����ÿ��ms��Ҫ��systickʱ����
}
