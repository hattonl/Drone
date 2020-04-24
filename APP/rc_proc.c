#include "rc_proc.h"

#define DEAD_BAND(value, mid, ban) \
    (((value) < (mid) - (ban))     \
         ? ((value) + (ban))       \
         : (((value) > (mid) + (ban)) ? ((value) - (ban)) : (mid)))

// u16 Rc_Pwm_Capt[CH_NUM] = {1500,1000,1500,1500};

void Rc_DataTrans(u16 Pwm_in[CH_NUM], u16 Pwm_out[CH_NUM]) {
    u8 i;
    for (i = 0; i < CH_NUM; i++)
        Pwm_out[i] = Pwm_in[i];
}

void rc_data_proc(u16 Tim3_Pwm_Capt[CH_NUM], u16 Rc_Pwm_Capt_out[CH_NUM]) {
    u8 i;
    static u16 Rc_Pwm_Capt_TEMP[CH_NUM] = {1500, 1000, 1500, 1500};

    for (i = 0; i < CH_NUM; i++) {
        if (Tim3_Pwm_Capt[i] > 2200 || Tim3_Pwm_Capt[i] < 800) {
            //通道i 产生错误
        } else {
            Rc_Pwm_Capt_TEMP[i] = Tim3_Pwm_Capt[i];
        }
    }

    for (i = 0; i < CH_NUM; i++) {
        Rc_Pwm_Capt_out[i] =
            DEAD_BAND(Rc_Pwm_Capt_TEMP[i], 1500, RC_DEAD_BAND_VALUE);
    }

    //	TIM1->CCR1 = Rc_Pwm_Capt[THR];				//1
    //	TIM1->CCR2 = Rc_Pwm_Capt[THR];				//2
    //	TIM1->CCR3 = Rc_Pwm_Capt[THR];				//3
    //	TIM1->CCR4 = Rc_Pwm_Capt[THR];				//4

    // printf("Rc_Pwm_Capt: %d  %d  %d  %d
    // \r\n",Rc_Pwm_Capt[YAW],Rc_Pwm_Capt[THR],Rc_Pwm_Capt[PIT],Rc_Pwm_Capt[ROL]);
}
