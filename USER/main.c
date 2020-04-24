#include "HMC5883L.h"
#include "MPU6050.h"
#include "hard.h"
#include "imu.h"
#include "includes.h"
#include "os_app_hooks.h"
#include "rc_proc.h"
#include "stm32f4xx.h"
#include "systim.h"
#include "tim1_pwm_out.h"
#include "tim3_capture.h"
#include "ultrasonic.h"
#include "usart2.h"

#define TASK_START_PRIO 3
#define TASK_START_STK_SIZE 512
extern OS_TCB TaskStartTCB;
extern CPU_STK TASK_START_STK[TASK_START_STK_SIZE];
extern void Task_Start(void *p_arg);

int main(void) {
    OS_ERR err;
    CPU_SR_ALLOC();
    hardware_init();

    OSInit(&err);         //��ʼ��UCOSIII
    OS_CRITICAL_ENTER();  //�����ٽ���

    //������ʼ����
    OSTaskCreate(
        (OS_TCB *)&TaskStartTCB,        //������ƿ�
        (CPU_CHAR *)"Task_Start",       //��������
        (OS_TASK_PTR)Task_Start,        //������
        (void *)0,                      //���ݸ��������Ĳ���
        (OS_PRIO)TASK_START_PRIO,       //�������ȼ�
        (CPU_STK *)&TASK_START_STK[0],  //�����ջ����ַ
        (CPU_STK_SIZE)TASK_START_STK_SIZE / 10,  //�����ջ�����λ
        (CPU_STK_SIZE)TASK_START_STK_SIZE,       //�����ջ��С
        (OS_MSG_QTY)0,  //�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
        (OS_TICK)0,  //��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
        (void *)0,  //�û�����Ĵ洢��
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,  //����ѡ��
        (OS_ERR *)&err);  //��Ÿú�������ʱ�ķ���ֵ

    OS_CRITICAL_EXIT();  //�˳��ٽ���

    OSStart(&err);  //����UCOSIII

    while (1) ;
}
