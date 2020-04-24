#include "tim3_capture.h"

#define TIM3_PERIOD_VALUE 0xffff
#define NVIC_TIM3_CAPT_P 2
#define NVIC_TIM3_CAPT_S 0

void TIM3_Cap_Init(u16 psc) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM3_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM3_PERIOD_VALUE;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);

    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
                 ENABLE);

    TIM_Cmd(TIM3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIM3_CAPT_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIM3_CAPT_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

u16 Tim3_Pwm_Capt[CH_NUM] = {1500, 1000, 1500, 1500};

void _TIM3_IRQHandler(void) {
    static u16 temp_cnt1[CH_NUM];
    static u16 temp_cnt2[CH_NUM];
    //应有检查遥控器丢失的信息

    if (TIM3->SR & TIM_IT_CC1) {
        TIM3->SR = ~TIM_IT_CC1;  // TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM3->SR = ~TIM_FLAG_CC1OF;
        if (GPIOC->IDR & GPIO_Pin_6) {
            temp_cnt1[YAW] = TIM_GetCapture1(TIM3);
        } else {
            temp_cnt2[YAW] = TIM_GetCapture1(TIM3);
            if (temp_cnt2[YAW] >= temp_cnt1[YAW])
                Tim3_Pwm_Capt[YAW] = temp_cnt2[YAW] - temp_cnt1[YAW];
            else
                Tim3_Pwm_Capt[YAW] =
                    TIM3_PERIOD_VALUE - temp_cnt1[YAW] + temp_cnt2[YAW] + 1;
        }
    }

    if (TIM3->SR & TIM_IT_CC2) {
        TIM3->SR = ~TIM_IT_CC2;
        TIM3->SR = ~TIM_FLAG_CC2OF;
        if (GPIOC->IDR & GPIO_Pin_7) {
            temp_cnt1[THR] = TIM_GetCapture2(TIM3);
        } else {
            temp_cnt2[THR] = TIM_GetCapture2(TIM3);
            if (temp_cnt2[THR] >= temp_cnt1[THR])
                Tim3_Pwm_Capt[THR] = temp_cnt2[THR] - temp_cnt1[THR];
            else
                Tim3_Pwm_Capt[THR] =
                    TIM3_PERIOD_VALUE - temp_cnt1[THR] + temp_cnt2[THR] + 1;
        }
    }

    if (TIM3->SR & TIM_IT_CC3) {
        TIM3->SR = ~TIM_IT_CC3;
        TIM3->SR = ~TIM_FLAG_CC3OF;
        if (GPIOC->IDR & GPIO_Pin_8) {
            temp_cnt1[PIT] = TIM_GetCapture3(TIM3);
        } else {
            temp_cnt2[PIT] = TIM_GetCapture3(TIM3);
            if (temp_cnt2[PIT] >= temp_cnt1[PIT])
                Tim3_Pwm_Capt[PIT] = temp_cnt2[PIT] - temp_cnt1[PIT];
            else
                Tim3_Pwm_Capt[PIT] =
                    TIM3_PERIOD_VALUE - temp_cnt1[PIT] + temp_cnt2[PIT] + 1;
        }
    }
    if (TIM3->SR & TIM_IT_CC4) {
        TIM3->SR = ~TIM_IT_CC4;
        TIM3->SR = ~TIM_FLAG_CC4OF;
        if (GPIOC->IDR & GPIO_Pin_9) {
            temp_cnt1[ROL] = TIM_GetCapture4(TIM3);
        } else {
            temp_cnt2[ROL] = TIM_GetCapture4(TIM3);
            if (temp_cnt2[ROL] >= temp_cnt1[ROL])
                Tim3_Pwm_Capt[ROL] = temp_cnt2[ROL] - temp_cnt1[ROL];
            else
                Tim3_Pwm_Capt[ROL] =
                    TIM3_PERIOD_VALUE - temp_cnt1[ROL] + temp_cnt2[ROL] + 1;
        }
    }
}

void TIM3_IRQHandler(void) {
    OSIntEnter();
    _TIM3_IRQHandler();
    OSIntExit();
}
/*end*/
