#include "MPU6050.h"
#include "my_delay.h"

#define delay_ms(x) my_delay_ms(x)
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void) {
    IIC_Init();                               //初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80);  //复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X03);  //唤醒MPU6050，设置CLKSEL,PLL,Z轴为参考  另一个为0x03
    delay_ms(10);
    MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00);  //加速度与陀螺仪解除休眠
    delay_ms(10);
    MPU_Set_Gyro_Fsr(3);  //陀螺仪传感器,±2000dps  //没有设置不自检
    delay_ms(10);
    MPU_Set_Accel_Fsr(2);  //加速度传感器,±2g  //最大量程ANO 设置为了 +/- 8g
    delay_ms(10);
    MPU_Set_Rate(1000);  //设置采样率50Hz   //ANO写入的是1000Hz 就是值为0
    delay_ms(10);
    MPU_Set_LPF(20);  // 20Hz低通滤波
    delay_ms(10);
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);  //关闭所有中断
    delay_ms(10);
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);  //不让MPU6050 控制AUXI2C，IIC主模式关闭
    delay_ms(10);
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);  //关闭FIFO
    delay_ms(10);
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X02);  //主控制器的I2C与	MPU6050的AUXI2C	直通。
    delay_ms(10);
    return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
// fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr) {
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);  //设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
// fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr) {
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);  //设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
// lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_LPF(u16 lpf) {
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data);  //设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
// rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate) {
    u8 data;
    if (rate > 1000) rate = 1000;
    if (rate < 4) rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);
    // return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半rate/2
    // //修改为5hz
    return data;
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void) {
    u8 buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
    ;
}
//得到陀螺仪值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz) {
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0) {
        *gx = (((u16)buf[0] << 8) | buf[1]);  // GX_TEM;
        *gy = (((u16)buf[2] << 8) | buf[3]);  // GY_TEM;
        *gz = (((u16)buf[4] << 8) | buf[5]);  // GZ_TEM;
    }
    return res;
    ;
}
//得到加速度值(原始值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az) {
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0) {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

//读取6050数据：加速度、温度、角速度
//返回值：0，成功
//			其他，错误代码
u8 MPU_ReadData(u8 Buf[]) {
    u8 res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 14, Buf);
    return res;
}

char Acc_CALIBRATE = 1;
char Gyro_CALIBRATE = 1;
float Gyro_tmp[3];
float Acc_Offset[3];
float Gyro_Offset[3];
u16 acc_sum_cnt = 0, gyro_sum_cnt = 0;
s32 sum_temp[7] = {0};
int32_t filter[6][10 + 1] = {0};
u8 dex = 0;
u8 i;
#define OFFSET_AV_NUM 50
// 6050数据处理
void MPU_DataProc(u8 *mpu_6050_buf, short *ax, short *ay, short *az, short *gx,
                  short *gy, short *gz) {
    int16_t ax0, ay0, az0, gx0, gy0, gz0;

    ax0 = ((((int16_t)mpu_6050_buf[0]) << 8) | mpu_6050_buf[1]);
    ay0 = ((((int16_t)mpu_6050_buf[2]) << 8) | mpu_6050_buf[3]);
    az0 = ((((int16_t)mpu_6050_buf[4]) << 8) | mpu_6050_buf[5]);

    gx0 = ((((int16_t)mpu_6050_buf[8]) << 8) | mpu_6050_buf[9]);
    gy0 = ((((int16_t)mpu_6050_buf[10]) << 8) | mpu_6050_buf[11]);
    gz0 = ((((int16_t)mpu_6050_buf[12]) << 8) | mpu_6050_buf[13]);

    if (Acc_CALIBRATE == 1) {
        acc_sum_cnt++;
        sum_temp[0] += ax0;
        sum_temp[1] += ay0;
        sum_temp[2] += az0 - 65536 / 16;  // +-8G
                                          // sum_temp[3] += mpu6050.Tempreature;

        if (acc_sum_cnt >= OFFSET_AV_NUM) {
            Acc_Offset[0] = sum_temp[0] / OFFSET_AV_NUM;
            Acc_Offset[1] = sum_temp[1] / OFFSET_AV_NUM;
            Acc_Offset[2] = sum_temp[2] / OFFSET_AV_NUM;
            // Acc_Temprea_Offset = sum_temp[3]/OFFSET_AV_NUM;
            acc_sum_cnt = 0;
            Acc_CALIBRATE = 0;
            // Param_SaveAccelOffset(&mpu6050.Acc_Offset);
            sum_temp[0] = sum_temp[1] = sum_temp[2] = sum_temp[3] = 0;
        }
    }

    if (Gyro_CALIBRATE) {
        gyro_sum_cnt++;
        sum_temp[4] += gx0;
        sum_temp[5] += gy0;
        sum_temp[6] += gz0;
        // sum_temp[3] += mpu6050.Tempreature;

        if (gyro_sum_cnt >= OFFSET_AV_NUM) {
            Gyro_Offset[0] = (float)sum_temp[4] / OFFSET_AV_NUM;
            Gyro_Offset[1] = (float)sum_temp[5] / OFFSET_AV_NUM;
            Gyro_Offset[2] = (float)sum_temp[6] / OFFSET_AV_NUM;
            // mpu6050.Gyro_Temprea_Offset = sum_temp[3]/OFFSET_AV_NUM;
            gyro_sum_cnt = 0;
            if (Gyro_CALIBRATE == 1)
                ;
            // Param_SaveGyroOffset(&mpu6050.Gyro_Offset);
            Gyro_CALIBRATE = 0;
            sum_temp[4] = sum_temp[5] = sum_temp[6] = sum_temp[3] = 0;
        }
    }

    //滑动窗口滤波
    filter[0][dex] = ax0;
    filter[1][dex] = ay0;
    filter[2][dex] = az0;

    filter[3][dex] = gx0;
    filter[4][dex] = gy0;
    filter[5][dex] = gz0;

    for (i = 0; i < 6; i++) {
        filter[i][10] = filter[i][0] + filter[i][1] + filter[i][2] +
                        filter[i][3] + filter[i][4] + filter[i][5] +
                        filter[i][6] + filter[i][7] + filter[i][8] +
                        filter[i][9];
    }

    *ax = filter[0][10] / 10 - Acc_Offset[0];
    *ay = filter[1][10] / 10 - Acc_Offset[1];
    *az = filter[2][10] / 10 - Acc_Offset[2];

    *gx = filter[3][10] / 10 - Gyro_Offset[0];
    *gy = filter[4][10] / 10 - Gyro_Offset[1];
    *gz = filter[5][10] / 10 - Gyro_Offset[2];

    dex++;
    if (dex >= 10) dex = 0;
}

// IIC连续写
// addr:器件地址
// reg:寄存器地址
// len:写入长度
// buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf) {
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);  //发送器件地址+写命令
    if (IIC_Wait_Ack())              //等待应答
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
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf) {
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);  //发送器件地址+写命令
    if (IIC_Wait_Ack())              //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);  //写寄存器地址
    IIC_Wait_Ack();      //等待应答
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1);  //发送器件地址+读命令
    IIC_Wait_Ack();                  //等待应答
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
u8 MPU_Write_Byte(u8 reg, u8 data) {
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0);  //发送器件地址+写命令
    if (IIC_Wait_Ack())                  //等待应答
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
u8 MPU_Read_Byte(u8 reg) {
    u8 res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0);  //发送器件地址+写命令
    IIC_Wait_Ack();                      //等待应答
    IIC_Send_Byte(reg);                  //写寄存器地址
    IIC_Wait_Ack();                      //等待应答
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 1);  //发送器件地址+读命令
    IIC_Wait_Ack();                      //等待应答
    res = IIC_Read_Byte(0);              //读取数据,发送nACK
    IIC_Stop();                          //产生一个停止条件
    return res;
}
