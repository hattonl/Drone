#include "HMC5883L.h"

#include "my_delay.h"

#define MAG_GAIN_1 (1.19047)
#define MAG_GAIN_2 (1.06382)

#define delay_ms(x) my_delay_ms(x)
u8 HMCInit() {
    if (HMC_Write_Byte(PA, 0x00)) return 1;  //正常测量
    delay_ms(200);
    if (HMC_Write_Byte(PB, 0x80)) return 1;  //±0.88G
    delay_ms(200);
    if (HMC_Write_Byte(Modle, 0x01)) return 1;
    delay_ms(200);
    return 0;
}

u8 HMC_Run() {
    if (HMC_Write_Byte(Modle, 0x01)) return 1;
    return 0;
}

u8 HMC_Write_Len(u8 reg, u8 len, u8 *buf) {
    u8 i;
    IIC_Start();
    IIC_Send_Byte(Write_HMC);  //发送器件地址+写命令
    if (IIC_Wait_Ack())        //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();      //等待应答
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(buf[i]);  //发送数据
        if (IIC_Wait_Ack())     //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}
// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 HMC_Read_Len(u8 reg, u8 len, u8 *buf) {
    IIC_Start();
    IIC_Send_Byte(Write_HMC);  //发送器件地址+写命令
    if (IIC_Wait_Ack())        //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();      //等待应答
    IIC_Start();
    IIC_Send_Byte(Read_HMC);  //发送器件地址+读命令
    IIC_Wait_Ack();           //等待应答
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);  //读数据,发送nACK
        else
            *buf = IIC_Read_Byte(1);  //读数据,发送ACK
        len--;
        buf++;
    }
    IIC_Stop();  //产生一个停止条件
    return 0;
}
// IIC写一个字节
// reg:寄存器地址
// data:数据
//返回值:0,正常
//    其他,错误代码
u8 HMC_Write_Byte(u8 reg, u8 data) {
    IIC_Start();
    IIC_Send_Byte(Write_HMC);  //发送器件地址+写命令
    if (IIC_Wait_Ack())        //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);   //写寄存器地址
    IIC_Wait_Ack();       //等待应答
    IIC_Send_Byte(data);  //发送数据
    if (IIC_Wait_Ack())   //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
// IIC读一个字节
// reg:寄存器地址
//返回值:读到的数据
u8 HMC_Read_Byte(u8 reg) {
    u8 res;
    IIC_Start();
    IIC_Send_Byte(Write_HMC);  //发送器件地址+写命令
    IIC_Wait_Ack();            //等待应答
    IIC_Send_Byte(reg);        //写寄存器地址
    IIC_Wait_Ack();            //等待应答
    IIC_Start();
    IIC_Send_Byte(Read_HMC);  //发送器件地址+读命令
    IIC_Wait_Ack();           //等待应答
    res = IIC_Read_Byte(0);   //读取数据,发送nACK
    IIC_Stop();               //产生一个停止条件
    return res;
}

float hmc_offset[3] = {0};
float hmc_val[3] = {0};
float Mag_Gain[3] = {0};
u8 Mag_CALIBRATED = 1;

void Read_HMC5883L(float *mx, float *my, float *mz) {
    int16_t hmc_temp[3];
    u8 BUF1[7] = {0};
    u8 tem;

    //校准用
    static float MagMAX[3] = {-100, -100, -100};
    static float MagMIN[3] = {100, 100, 100};
    static float MagSum[3];
    static uint16_t cnt_m = 0;

    tem = HMC_Read_Byte(HMC5883L_StatusRegister);
    while (!(tem & 0x01)) HMC_Read_Byte(HMC5883L_StatusRegister);

    //	BUF1[1]=HMC_Read_Byte(XMSB);//OUT_X_H  0x03
    //	BUF1[2]=HMC_Read_Byte(XLSB);//OUT_X_L  0x04
    //	BUF1[3]=HMC_Read_Byte(YMSB);//OUT_Y_L_A 0x07
    //	BUF1[4]=HMC_Read_Byte(YLSB);//OUT_Y_H_A  0x08
    //  BUF1[5]=HMC_Read_Byte(ZMSB);//OUT_Z_L_A   0x05
    //	BUF1[6]=HMC_Read_Byte(ZLSB);//OUT_Z_H_A		0x06
    //
    //	hmc_temp[0] = (BUF1[1] << 8) | BUF1[2];
    //	hmc_temp[1] = (BUF1[3] << 8) | BUF1[4];
    //  hmc_temp[2] = (BUF1[5] << 8) | BUF1[6];

    HMC_Read_Len(XMSB, 6, (u8 *)(&BUF1[1]));
    //
    hmc_temp[0] = (BUF1[1] << 8) | BUF1[2];
    hmc_temp[1] = (BUF1[5] << 8) | BUF1[6];
    hmc_temp[2] = (BUF1[3] << 8) | BUF1[4];

    if (hmc_temp[0] & 0x8000) {
        hmc_temp[0] &= 0x7fff;
        (hmc_temp[0])--;
        hmc_temp[0] ^= 0x7fff;
        hmc_temp[0] = -1 * (hmc_temp[0]);
    }
    if (hmc_temp[1] & 0x8000) {
        hmc_temp[1] &= 0x7fff;
        (hmc_temp[1])--;
        hmc_temp[1] ^= 0x7fff;
        hmc_temp[1] = -1 * (hmc_temp[1]);
    }
    if (hmc_temp[2] & 0x8000) {
        hmc_temp[2] &= 0x7fff;
        (hmc_temp[2])--;
        hmc_temp[2] ^= 0x7fff;
        hmc_temp[2] = -1 * (hmc_temp[2]);
    }

    hmc_val[0] = (hmc_temp[0] - (-78));  //-78
    hmc_val[1] = (hmc_temp[1] - 17);     // 17
    hmc_val[2] = (hmc_temp[2] - (-22));  //-22

    //	hmc_val[0] = (hmc_temp[0] - hmc_offset[0]);//-78
    //	hmc_val[1] = (hmc_temp[1] - hmc_offset[1]);//17
    //	hmc_val[2] = (hmc_temp[2] - hmc_offset[2]);//-22

    //校准

    if (Mag_CALIBRATED) {
        if (ABS(hmc_temp[0]) < 1000 && ABS(hmc_temp[1]) < 1000 &&
            ABS(hmc_temp[2]) < 1000) {
            MagMAX[0] = MAX(hmc_temp[0], MagMAX[0]);
            MagMAX[1] = MAX(hmc_temp[1], MagMAX[1]);
            MagMAX[2] = MAX(hmc_temp[2], MagMAX[2]);

            MagMIN[0] = MIN(hmc_temp[0], MagMIN[0]);
            MagMIN[1] = MIN(hmc_temp[1], MagMIN[1]);
            MagMIN[2] = MIN(hmc_temp[2], MagMIN[2]);

            if (cnt_m == 2000) {
                hmc_offset[0] = (int16_t)((MagMAX[0] + MagMIN[0]) * 0.5f);
                hmc_offset[1] = (int16_t)((MagMAX[1] + MagMIN[1]) * 0.5f);
                hmc_offset[2] = (int16_t)((MagMAX[2] + MagMIN[2]) * 0.5f);

                MagSum[0] = MagMAX[0] - MagMIN[0];
                MagSum[1] = MagMAX[1] - MagMIN[1];
                MagSum[2] = MagMAX[2] - MagMIN[2];

                Mag_Gain[1] = MagSum[0] / MagSum[1];
                Mag_Gain[2] = MagSum[0] / MagSum[2];

                // Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//保存数据
                cnt_m = 0;
                Mag_CALIBRATED = 0;
            }
        }
        cnt_m++;

    } else {
    }

    // Mag_Gain[1] = (float)MAG_GAIN_1;
    // Mag_Gain[2] = (float)MAG_GAIN_2;

    hmc_val[0] *= 1;
    hmc_val[1] *= (float)1.05866671;   // 1.05866671
    hmc_val[2] *= (float)0.985111654;  // 0.985111654

    //	hmc_val[0] *= 1 ;
    //	hmc_val[1] *= Mag_Gain[1];//1.05866671
    //	hmc_val[2] *= Mag_Gain[2];//0.985111654

    *mx = hmc_val[0];
    *my = hmc_val[1];
    *mz = hmc_val[2];

    HMC_Run();  //
}
