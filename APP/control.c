#include "control.h"

StrPIDContrl PitCont;
StrPIDContrl RolCont;
StrPIDContrl YawCont;

#define MAX_CTRL_ANGLE 25.0f
#define MAX_CTRL_ASPEED 300.0f
#define MAX_CTRL_YAW_SPEED 150.0f

#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

int16_t motor[4];
int16_t motor_add[4] = {0};
uint16_t Moto_Limit = 0;

void Control_Para_Init(void) {
    PitCont.Outer.P = 2.5;
    PitCont.Outer.I = 0;
    PitCont.Outer.D = 0;

    PitCont.Inner.P = 1.2;
    PitCont.Inner.I = 1;
    PitCont.Inner.D = 0.08;

    RolCont.Outer.P = 2.5;
    RolCont.Outer.I = 0;
    RolCont.Outer.D = 0;

    RolCont.Inner.P = 1.2;
    RolCont.Inner.I = 1.0;
    RolCont.Inner.D = 0.08;

    YawCont.Outer.P = 5.0;
    YawCont.Outer.I = 0;
    YawCont.Outer.D = 0;

    YawCont.Inner.P = 5.0;
    YawCont.Inner.I = 1.2;
    YawCont.Inner.D = 0;
}

void Control_Outer(float T, u16 CH[CH_NUM], float myRoll, float myPitch,
                   float myYaw) {
    //得到期望角度
    PitCont.Outer.Desired = -MAX_CTRL_ANGLE * ((CH[PIT] - 1500) / 500.0f);
    RolCont.Outer.Desired = MAX_CTRL_ANGLE * ((CH[ROL] - 1500) / 500.0f);

    if (CH[THR] > 1100)
        YawCont.Outer.Desired +=
            MAX_CTRL_YAW_SPEED * ((CH[YAW] - 1500) / 500.0f) * T;
    else
        YawCont.Outer.Desired += 1 * 3.14 * T * (myYaw - YawCont.Outer.Desired);

    //计算误差
    PitCont.Outer.Error = To_180_degrees(PitCont.Outer.Desired - myPitch);
    RolCont.Outer.Error = To_180_degrees(RolCont.Outer.Desired - myRoll);
    YawCont.Outer.Error = To_180_degrees(YawCont.Outer.Desired - myYaw);

    if (PitCont.Outer.Error > 5 || PitCont.Outer.Error < -5) {
        //积分
        PitCont.Outer.Integ += PitCont.Outer.Error * T;
    } else {
        PitCont.Outer.Integ = 0;
    }

    if (RolCont.Outer.Error > 5 || RolCont.Outer.Error < -5) {
        //积分
        RolCont.Outer.Integ += RolCont.Outer.Error * T;
    } else {
        RolCont.Outer.Integ = 0;
    }

    if (YawCont.Outer.Error > 5 || YawCont.Outer.Error < -5) {
        //积分
        YawCont.Outer.Integ += YawCont.Outer.Error * T;
    } else {
        YawCont.Outer.Integ = 0;
    }

    //积分限
    PitCont.Outer.Integ = LIMIT(PitCont.Outer.Integ, -100, 100);
    RolCont.Outer.Integ = LIMIT(RolCont.Outer.Integ, -100, 100);
    YawCont.Outer.Integ = LIMIT(YawCont.Outer.Integ, -100, 100);
    //计算微分
    PitCont.Outer.Deriv = (PitCont.Outer.Error - PitCont.Outer.PreError) / T;
    RolCont.Outer.Deriv = (RolCont.Outer.Error - RolCont.Outer.PreError) / T;
    YawCont.Outer.Deriv = (YawCont.Outer.Error - YawCont.Outer.PreError) / T;
    // PID分别输出值,方便调试
    PitCont.Outer.out_P = PitCont.Outer.P * PitCont.Outer.Error;
    RolCont.Outer.out_P = RolCont.Outer.P * RolCont.Outer.Error;
    YawCont.Outer.out_P = YawCont.Outer.P * YawCont.Outer.Error;

    PitCont.Outer.out_I = PitCont.Outer.I * PitCont.Outer.Integ;
    RolCont.Outer.out_I = RolCont.Outer.I * RolCont.Outer.Integ;
    YawCont.Outer.out_I = YawCont.Outer.I * YawCont.Outer.Integ;

    PitCont.Outer.out_D = PitCont.Outer.D * PitCont.Outer.Deriv;
    RolCont.Outer.out_D = RolCont.Outer.D * RolCont.Outer.Deriv;
    YawCont.Outer.out_D = YawCont.Outer.D * YawCont.Outer.Deriv;

    //外环输出值
    PitCont.Outer.Output =
        PitCont.Outer.out_P + PitCont.Outer.out_I + PitCont.Outer.out_D;
    RolCont.Outer.Output =
        RolCont.Outer.out_P + RolCont.Outer.out_I + RolCont.Outer.out_D;
    YawCont.Outer.Output =
        YawCont.Outer.out_P + YawCont.Outer.out_I + YawCont.Outer.out_D;

    //保存上次的值
    PitCont.Outer.PreError = PitCont.Outer.Error;
    RolCont.Outer.PreError = RolCont.Outer.Error;
    YawCont.Outer.PreError = YawCont.Outer.Error;
}

void Control_Inner(float T, u16 CH[CH_NUM], float GryoX, float GryoY,
                   float GryoZ, int16_t motor_ts[4]) {
    PitCont.Inner.Desired =
        LIMIT(-PitCont.Outer.Output, -MAX_CTRL_ASPEED, MAX_CTRL_ASPEED);
    // PitCont.Inner.Desired = MAX_CTRL_ASPEED * ((CH[PIT] - 1500)/500.0f);
    RolCont.Inner.Desired =
        LIMIT(RolCont.Outer.Output, -MAX_CTRL_ASPEED, MAX_CTRL_ASPEED);
    // RolCont.Inner.Desired = MAX_CTRL_ASPEED * ((CH[ROL] - 1500)/500.0f);
    YawCont.Inner.Desired =
        LIMIT(-YawCont.Outer.Output, -MAX_CTRL_YAW_SPEED, MAX_CTRL_YAW_SPEED);
    // YawCont.Inner.Desired = MAX_CTRL_YAW_SPEED * ((CH[YAW] - 1500)/500.0f);

    RolCont.Inner.Error = (RolCont.Inner.Desired - GryoX);
    PitCont.Inner.Error = (PitCont.Inner.Desired - GryoY);
    YawCont.Inner.Error = (YawCont.Inner.Desired - GryoZ);

    //积分
    PitCont.Inner.Integ += PitCont.Inner.Error * T;
    RolCont.Inner.Integ += RolCont.Inner.Error * T;
    YawCont.Inner.Integ += YawCont.Inner.Error * T;

    if (PitCont.Inner.Integ < 20 || PitCont.Inner.Integ > -20) {
        PitCont.Inner.Integ += PitCont.Inner.Error * T;
    } else {
        PitCont.Inner.Integ = 0;
    }

    if (RolCont.Inner.Integ < 20 || RolCont.Inner.Integ > -20) {
        RolCont.Inner.Integ += RolCont.Inner.Error * T;
    } else {
        RolCont.Inner.Integ = 0;
    }

    if (YawCont.Inner.Integ < 20 || YawCont.Inner.Integ > -20) {
        YawCont.Inner.Integ += YawCont.Inner.Error * T;
    } else {
        YawCont.Inner.Integ = 0;
    }

    //积分限
    PitCont.Inner.Integ = LIMIT(PitCont.Inner.Integ, -50, 50);
    RolCont.Inner.Integ = LIMIT(RolCont.Inner.Integ, -50, 50);
    YawCont.Inner.Integ = LIMIT(YawCont.Inner.Integ, -50, 50);

    //计算微分
    PitCont.Inner.Deriv = (PitCont.Inner.Error - PitCont.Inner.PreError) / T;
    RolCont.Inner.Deriv = (RolCont.Inner.Error - RolCont.Inner.PreError) / T;
    YawCont.Inner.Deriv = (YawCont.Inner.Error - YawCont.Inner.PreError) / T;

    // PID分别输出值,方便调试
    PitCont.Inner.out_P = PitCont.Inner.P * PitCont.Inner.Error;
    RolCont.Inner.out_P = RolCont.Inner.P * RolCont.Inner.Error;
    YawCont.Inner.out_P = YawCont.Inner.P * YawCont.Inner.Error;

    PitCont.Inner.out_I = PitCont.Inner.I * PitCont.Inner.Integ;
    RolCont.Inner.out_I = RolCont.Inner.I * RolCont.Inner.Integ;
    YawCont.Inner.out_I = YawCont.Inner.I * YawCont.Inner.Integ;

    PitCont.Inner.out_D = PitCont.Inner.D * PitCont.Inner.Deriv;
    RolCont.Inner.out_D = RolCont.Inner.D * RolCont.Inner.Deriv;
    YawCont.Inner.out_D = YawCont.Inner.D * YawCont.Inner.Deriv;

    //外环输出值
    PitCont.Inner.Output =
        -(PitCont.Inner.out_P + PitCont.Inner.out_I + PitCont.Inner.out_D);
    RolCont.Inner.Output =
        -(RolCont.Inner.out_P + RolCont.Inner.out_I + RolCont.Inner.out_D);
    YawCont.Inner.Output =
        (YawCont.Inner.out_P + YawCont.Inner.out_I + YawCont.Inner.out_D);

    PitCont.Inner.Output = LIMIT(PitCont.Inner.Output, -250, 250);
    RolCont.Inner.Output = LIMIT(RolCont.Inner.Output, -250, 250);
    YawCont.Inner.Output = LIMIT(YawCont.Inner.Output, -250, 250);
    //保存上次的值
    PitCont.Inner.PreError = PitCont.Inner.Error;
    RolCont.Inner.PreError = RolCont.Inner.Error;
    YawCont.Inner.PreError = YawCont.Inner.Error;

    Moto_Limit = (CH[THR] - 1100) / 2;

    motor_add[0] = (int16_t)(+RolCont.Inner.Output - PitCont.Inner.Output +
                             YawCont.Inner.Output);
    motor_add[1] = (int16_t)(-RolCont.Inner.Output - PitCont.Inner.Output -
                             YawCont.Inner.Output);
    motor_add[2] = (int16_t)(-RolCont.Inner.Output + PitCont.Inner.Output +
                             YawCont.Inner.Output);
    motor_add[3] = (int16_t)(+RolCont.Inner.Output + PitCont.Inner.Output -
                             YawCont.Inner.Output);

    motor_add[0] = LIMIT(motor_add[0], -Moto_Limit, Moto_Limit);
    motor_add[1] = LIMIT(motor_add[1], -Moto_Limit, Moto_Limit);
    motor_add[2] = LIMIT(motor_add[2], -Moto_Limit, Moto_Limit);
    motor_add[3] = LIMIT(motor_add[3], -Moto_Limit, Moto_Limit);

    //电机输出
    if (CH[THR] > 1100) {
        motor[0] = (int16_t)(CH[THR] + motor_add[0] - 100);
        motor[1] = (int16_t)(CH[THR] + motor_add[1] - 100);
        motor[2] = (int16_t)(CH[THR] + motor_add[2] - 100);
        motor[3] = (int16_t)(CH[THR] + motor_add[3] - 100);

    } else {
        motor[0] = 1000;
        motor[1] = 1000;
        motor[2] = 1000;
        motor[3] = 1000;
        PitCont.Inner.Integ = 0;
        PitCont.Outer.Integ = 0;
        RolCont.Inner.Integ = 0;
        RolCont.Outer.Integ = 0;
        YawCont.Outer.Integ = 0;
        YawCont.Inner.Integ = 0;
    }

    //电机限
    motor[0] = LIMIT(motor[0], 1000, 2000);
    motor[1] = LIMIT(motor[1], 1000, 2000);
    motor[2] = LIMIT(motor[2], 1000, 2000);
    motor[3] = LIMIT(motor[3], 1000, 2000);

    //输出
    TIM1->CCR1 = motor[0];  // 1
    TIM1->CCR2 = motor[1];  // 2
    TIM1->CCR3 = motor[2];  // 3
    TIM1->CCR4 = motor[3];  // 4

    motor_ts[0] = motor[0];
    motor_ts[1] = motor[1];
    motor_ts[2] = motor[2];
    motor_ts[3] = motor[3];
}
