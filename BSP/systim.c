#include "systim.h"
// use only os
#include "includes.h"

static u8 fac_us = 0;   // us延时倍乘数
static u16 fac_ms = 0;  // ms延时倍乘数,在os下,代表每个节拍的ms数

extern void PutTaskRun(void);

void SysTick_Handler(void) {
    if (OSRunning == 1)  // OS开始跑了,才执行正常的调度处理
    {
        OSIntEnter();  //进入中断
        PutTaskRun();
        OSTimeTick();  //调用ucos的时钟服务程序
        OSIntExit();  //触发任务切换软中断
    }
}

void delay_init(u8 SYSCLK) {
    u32 reload;

    SysTick_CLKSourceConfig(
        SysTick_CLKSource_HCLK_Div8);  // SYSTICK使用外部时钟源
    fac_us = SYSCLK / 8;  //不论是否使用OS,fac_us都需要使用

    reload = SYSCLK / 8;  //每秒钟的计数次数 单位为K
    reload *= 1000000 / OSCfg_TickRate_Hz;
			//根据delay_ostickspersec设定溢出时间
            // reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右
    fac_ms = 1000 / OSCfg_TickRate_Hz;  //代表OS可以延时的最少单位
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  //开启SYSTICK中断
    SysTick->LOAD = reload;  //每1/OS_TICKS_PER_SEC秒中断一次
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //开启SYSTICK

    fac_ms = (u16)fac_us * 1000;  //非OS下,代表每个ms需要的systick时钟数
}
