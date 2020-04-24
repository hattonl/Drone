#include "tim2.h"

//中断周期1ms
//用于向具有严格周期任务发送同步

#define TIM2_CYCLE_TIME 1000
#define NVIC_TIM2_CAPT_P 2
#define NVIC_TIM2_CAPT_S 1

extern OS_TCB TaskControlTCB;
extern OS_TCB TaskAttitudeTCB;
void TIM2_Init(u16 psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM2_CYCLE_TIME - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIM2_CAPT_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIM2_CAPT_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void PutTaskRun(void) {
    static u8 i = 0;
    OS_ERR err;
    if (i == 1) {
        //发布信号量给Task_Control 周期2ms
        OSTaskSemPost((OS_TCB *)&TaskControlTCB, (OS_OPT)OS_OPT_POST_NO_SCHED,
                      (OS_ERR *)&err);
    } else {
        //发布信号量给Task_Attitude 周期2ms
        OSTaskSemPost((OS_TCB *)&TaskAttitudeTCB, (OS_OPT)OS_OPT_POST_NO_SCHED,
                      (OS_ERR *)&err);
    }

    i++;
    if (i > 1) i = 0;
}

void _TIM2_IRQHandler(void) {
    static u8 i = 0;
    OS_ERR err;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        if (OSRunning == 1) {
            if (i == 1) {
                //发布信号量给Task_Control 周期2ms
                OSTaskSemPost((OS_TCB *)&TaskControlTCB,
                              (OS_OPT)OS_OPT_POST_NONE, (OS_ERR *)&err);
            } else {
                //发布信号量给Task_Attitude 周期2ms
                OSTaskSemPost((OS_TCB *)&TaskAttitudeTCB,
                              (OS_OPT)OS_OPT_POST_NONE, (OS_ERR *)&err);
            }

            i++;
            if (i > 1) i = 0;
        }
    }

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    //清楚中断标志位
}

void TIM2_IRQHandler(void) {
    OSIntEnter();
    _TIM2_IRQHandler();
    OSIntExit();
}
/*end*/
