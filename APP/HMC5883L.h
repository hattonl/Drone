#ifndef _HMC5883L_H
#define _HMC5883L_H
#include "I2Cdev.h"
#include "stm32f4xx.h"

#define Write_HMC 0x3C
#define Read_HMC 0x3D

#define PA 0x00
#define PB 0x01

#define Modle 0x02
#define XMSB 0x03
#define XLSB 0x04
#define ZMSB 0x05
#define ZLSB 0x06
#define YMSB 0x07
#define YLSB 0x08
#define HMC5883L_StatusRegister 0x09
#define HMC5883L_ID_A 0x0A
#define HMC5883L_ID_B 0x0B
#define HMC5883L_ID_C 0x0C

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

u8 HMCInit(void);
u8 HMC_Run(void);
u8 HMC_Write_Len(u8 reg, u8 len, u8 *buf);
u8 HMC_Read_Len(u8 reg, u8 len, u8 *buf);
u8 HMC_Write_Byte(u8 reg, u8 data);
u8 HMC_Read_Byte(u8 reg);
void Read_HMC5883L(float *mx, float *my, float *mz);

#endif
