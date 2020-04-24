#ifndef _I2CDEV_H
#define _I2CDEV_H

#include "head.h"
#include "stm32f4xx.h"
// IO方向设置
#define SDA_IN()                         \
    {                                    \
        GPIOB->MODER &= ~(3 << (5 * 2)); \
        GPIOB->MODER |= 0 << 5 * 2;      \
    }  // PB5输入模式
#define SDA_OUT()                        \
    {                                    \
        GPIOB->MODER &= ~(3 << (5 * 2)); \
        GPIOB->MODER |= 1 << 5 * 2;      \
    }  // PB5输出模式
// IO操作函数
#define IIC_SCL PBout(4)  // SCL
#define IIC_SDA PBout(5)  // SDA
#define READ_SDA PBin(5)  //输入SDA

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);

#endif
