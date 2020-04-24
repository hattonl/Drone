#ifndef __CONTROL_H
#define __CONTROL_H

#include "mymath.h"
#include "stm32f4xx.h"
#include "tim3_capture.h"

typedef struct {
    float P;
    float I;
    float D;
    float out_P;
    float out_I;
    float out_D;
    float Desired;
    float Error;
    float PreError;
    float Integ;
    float Deriv;
    float iLimit;
    float Output;

} StrPID;

typedef struct {
    StrPID Inner;
    StrPID Outer;

} StrPIDContrl;

void Control_Para_Init(void);
void Control_Outer(float T, u16 CH[CH_NUM], float myRoll, float myPitch,
                   float myYaw);
void Control_Inner(float T, u16 CH[CH_NUM], float GryoX, float GryoY,
                   float GryoZ, int16_t motor_ts[4]);

#endif
