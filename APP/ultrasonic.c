#include "ultrasonic.h"

#include "includes.h"
#include "my_delay.h"
// using tim4
void Ultrasonic_Init() {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB,
                           ENABLE);  // PC10 PB6

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIM4_ULTAR_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIM4_ULTAR_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // TRIG
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  // ECHO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

    TIM_TimeBaseStructure.TIM_Prescaler = 100 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TIM4_PERIOD_VALUE;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    /* TIM enable counter */
    TIM_Cmd(TIM4, ENABLE);

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
}

s8 ultra_start_f = -1;
u8 ultra_time;
u8 ultra_ok = 0;
void Ultra_Start() {
    if (ultra_start_f == -1)  // 100ms//改用发送中断
    {
        GPIOC->BSRRL = GPIO_Pin_10;
        my_delay_us(15);
        GPIOC->BSRRH = GPIO_Pin_10;
        //发10us高电平
        ultra_start_f = 1;
    }
}

u16 ultra_distance, ultra_distance_old;
s16 ultra_delta;
void Ultra_Get(u16 com_data) {
    if (ultra_start_f == 1) {
        ultra_distance = 0.17 * com_data;
        ultra_start_f = 0;
        ultra_ok = 1;
    }

    ultra_delta = ultra_distance - ultra_distance_old;

    ultra_distance_old = ultra_distance;
}

u16 ultra_get_data = 0;
void _TIM4_IRQHandler(void) {
    static u16 temp_cnt1, temp_cnt1_2;

    if (TIM4->SR & TIM_IT_CC1) {
        TIM4->SR = ~TIM_IT_CC1;  // TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        TIM4->SR = ~TIM_FLAG_CC1OF;
        if (GPIOB->IDR & GPIO_Pin_6) {
            temp_cnt1 = TIM_GetCapture1(TIM4);
        } else {
            temp_cnt1_2 = TIM_GetCapture1(TIM4);
            if (temp_cnt1_2 >= temp_cnt1)
                ultra_get_data = temp_cnt1_2 - temp_cnt1;
            else
                ultra_get_data = 0xffff - temp_cnt1 + temp_cnt1_2 + 1;

            ultra_start_f = -1;
            // Ultra_Get(Rc_Pwm_In[4]);
        }
    }
}

void TIM4_IRQHandler(void) {
    OSIntEnter();
    _TIM4_IRQHandler();
    OSIntExit();
}
