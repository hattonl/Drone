#ifndef __APPLICATION_H
#define __APPLICATION_H

#include "stm32f4xx.h"

enum PID_SELE { XO, XI, YO, YI, ZO, ZI, NO };

typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

typedef struct {
    xyz_f_t Gyro;
    xyz_f_t Acce;
    xyz_f_t Mag;
    float Rol;
    float Pit;
    float Yaw;
} StGy86_t;

typedef struct {
    float P;
    float I;
    float D;
    u8 opt;
    u8 is;
} Spid_t;

//**************支持数据传输的联合体*****************//
typedef union {
    float f_m;
    unsigned char u8_arr_m[4];
} U_Trans_f_t;

typedef union {
    u16 u16_m;
    unsigned char u8_arr_t[2];
} U_Trans_u16_t;

typedef union {
    int16_t int16_m;
    unsigned char u8_arr_t[2];
} U_Trans_int16_t;

//**************支持数据传输的结构体*****************//
typedef struct {
    U_Trans_f_t Rol;
    U_Trans_f_t Pit;
    U_Trans_f_t Yaw;

    U_Trans_f_t Gx;
    U_Trans_f_t Gy;
    U_Trans_f_t Gz;

    U_Trans_f_t Ax;
    U_Trans_f_t Ay;
    U_Trans_f_t Az;

} S_Trans_gy86_t;

#endif
