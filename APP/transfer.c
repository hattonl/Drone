#include "transfer.h"

void Usart2_Send(unsigned char *DataToSend, u8 data_num);
void ANO_DT_Send_Data(u8 *dataToSend, u8 length);
void ADD_TO_DMA(u8 *dataToSend, u8 length);

#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

extern u32 count;
extern u8 usart2_dma_buf[200];

u8 data_to_send[50];  //发送数据缓存

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw,
                        s32 alt, u8 fly_model, u8 armed) {
    u8 _cnt = 0;
    u8 sum = 0, i;
    vs16 _temp;
    vs32 _temp2 = alt;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    _temp = (int)(angle_rol * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_pit * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++) sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ADD_TO_DMA(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z,
                        s16 m_x, s16 m_y, s16 m_z) {
    u8 _cnt = 0;
    u8 sum = 0, i;
    vs16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;

    _temp = a_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    /////////////////////////////////////////
    _temp = 0;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++) sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ADD_TO_DMA(data_to_send, _cnt);
}

void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2,
                        u16 aux3, u16 aux4, u16 aux5, u16 aux6) {
    u8 _cnt = 0;
    u8 sum = 0, i;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(thr);
    data_to_send[_cnt++] = BYTE0(thr);
    data_to_send[_cnt++] = BYTE1(yaw);
    data_to_send[_cnt++] = BYTE0(yaw);
    data_to_send[_cnt++] = BYTE1(rol);
    data_to_send[_cnt++] = BYTE0(rol);
    data_to_send[_cnt++] = BYTE1(pit);
    data_to_send[_cnt++] = BYTE0(pit);
    data_to_send[_cnt++] = BYTE1(aux1);
    data_to_send[_cnt++] = BYTE0(aux1);
    data_to_send[_cnt++] = BYTE1(aux2);
    data_to_send[_cnt++] = BYTE0(aux2);
    data_to_send[_cnt++] = BYTE1(aux3);
    data_to_send[_cnt++] = BYTE0(aux3);
    data_to_send[_cnt++] = BYTE1(aux4);
    data_to_send[_cnt++] = BYTE0(aux4);
    data_to_send[_cnt++] = BYTE1(aux5);
    data_to_send[_cnt++] = BYTE0(aux5);
    data_to_send[_cnt++] = BYTE1(aux6);
    data_to_send[_cnt++] = BYTE0(aux6);

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++) sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ADD_TO_DMA(data_to_send, _cnt);
}

void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6,
                         u16 m_7, u16 m_8) {
    u8 _cnt = 0;
    u8 sum = 0, i;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x06;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE1(m_1);
    data_to_send[_cnt++] = BYTE0(m_1);
    data_to_send[_cnt++] = BYTE1(m_2);
    data_to_send[_cnt++] = BYTE0(m_2);
    data_to_send[_cnt++] = BYTE1(m_3);
    data_to_send[_cnt++] = BYTE0(m_3);
    data_to_send[_cnt++] = BYTE1(m_4);
    data_to_send[_cnt++] = BYTE0(m_4);
    data_to_send[_cnt++] = BYTE1(m_5);
    data_to_send[_cnt++] = BYTE0(m_5);
    data_to_send[_cnt++] = BYTE1(m_6);
    data_to_send[_cnt++] = BYTE0(m_6);
    data_to_send[_cnt++] = BYTE1(m_7);
    data_to_send[_cnt++] = BYTE0(m_7);
    data_to_send[_cnt++] = BYTE1(m_8);
    data_to_send[_cnt++] = BYTE0(m_8);

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++) sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ADD_TO_DMA(data_to_send, _cnt);
}

void ANO_DT_Send_Speed(float x_s, float y_s, float z_s) {
    u8 _cnt = 0;
    u8 sum = 0, i;
    vs16 _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x0B;
    data_to_send[_cnt++] = 0;

    _temp = (int)(x_s);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(y_s);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(z_s);  // dm/s
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;

    for (i = 0; i < _cnt; i++) sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ADD_TO_DMA(data_to_send, _cnt);
}

void ADD_TO_DMA(u8 *dataToSend, u8 length) {
    u8 i;
    for (i = 0; i < length; i++) {
        usart2_dma_buf[count++] = *(dataToSend + i);
    }
}

void ANO_DT_Send_Data(u8 *dataToSend, u8 length) {
    Usart2_Send(data_to_send, length);
}

void Usart2_Send(unsigned char *DataToSend, u8 data_num) {
    u8 i;
    for (i = 0; i < data_num; i++) {
        while ((USART2->SR & 0X40) == 0)
            ;  //循环发送,直到发送完毕
        USART2->DR = (u8) * (DataToSend + i);
    }
}

//	for(i=0;i<data_num;i++)
//	{
//		TxBuffer[count++] = *(DataToSend+i);
//	}

//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //打开发送中断
//	}
//
