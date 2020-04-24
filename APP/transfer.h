#ifndef __TRANSFER_H
#define __TRANSFER_H

#include "stm32f4xx.h"

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw,
                        s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z,
                        s16 m_x, s16 m_y, s16 m_z);
void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2,
                        u16 aux3, u16 aux4, u16 aux5, u16 aux6);
void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6,
                         u16 m_7, u16 m_8);
void ANO_DT_Send_Speed(float x_s, float y_s, float z_s);
#endif
