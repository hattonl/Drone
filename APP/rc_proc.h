#ifndef __RC_PROC_H
#define __RC_PROC_H

#include "stm32f4xx.h"
#include "tim3_capture.h"

#define DEAD_BAND(value, mid, ban) \
    (((value) < (mid) - (ban))     \
         ? ((value) + (ban))       \
         : (((value) > (mid) + (ban)) ? ((value) - (ban)) : (mid)))

#define RC_DEAD_BAND_VALUE (50)

void rc_data_proc(u16 Tim3_Pwm_Capt[CH_NUM], u16 Rc_Pwm_Capt[CH_NUM]);
void Rc_DataTrans(u16 Pwm_in[CH_NUM], u16 Pwm_out[CH_NUM]);

#endif
