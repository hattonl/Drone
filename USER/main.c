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

    OSInit(&err);         //初始化UCOSIII
    OS_CRITICAL_ENTER();  //进入临界区

    //创建开始任务
    OSTaskCreate(
        (OS_TCB *)&TaskStartTCB,        //任务控制块
        (CPU_CHAR *)"Task_Start",       //任务名字
        (OS_TASK_PTR)Task_Start,        //任务函数
        (void *)0,                      //传递给任务函数的参数
        (OS_PRIO)TASK_START_PRIO,       //任务优先级
        (CPU_STK *)&TASK_START_STK[0],  //任务堆栈基地址
        (CPU_STK_SIZE)TASK_START_STK_SIZE / 10,  //任务堆栈深度限位
        (CPU_STK_SIZE)TASK_START_STK_SIZE,       //任务堆栈大小
        (OS_MSG_QTY)0,  //任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
        (OS_TICK)0,  //当使能时间片轮转时的时间片长度，为0时为默认长度，
        (void *)0,  //用户补充的存储区
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,  //任务选项
        (OS_ERR *)&err);  //存放该函数错误时的返回值

    OS_CRITICAL_EXIT();  //退出临界区

    OSStart(&err);  //开启UCOSIII

    while (1) ;
}
