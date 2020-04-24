#ifndef _IMU_H_
#define _IMU_H_

#include "math.h"
#include "mymath.h"
#include "stm32f4xx.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))
void IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay,
               float az, float mx, float my, float mz, float *rol, float *pit,
               float *yaw);
extern float Roll, Pitch, Yaw;
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
                           float az, float *rol, float *pit, float *yaw);

#endif
