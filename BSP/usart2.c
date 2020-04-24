#include "usart2.h"

#include "includes.h"
#include "stdio.h"

#define NVIC_USART2_P 3
#define NVIC_USART2_S 3

extern OS_MEM UsartMemory;
extern OS_TCB TaskReceDataTCB;

#if 1
#pragma import(__use_no_semihosting)
typedef struct __FILE FILE;
//标准库需要的支持函数
struct __FILE {
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
_sys_exit(int x) { x = x; }
//重定义fputc函数
int fputc(int ch, FILE *f) {
    while ((USART2->SR & 0X40) == 0)
        ;  //循环发送,直到发送完毕
    USART2->DR = (u8)ch;
    return ch;
}
#endif

void Usart2_init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);

    USART_ClearFlag(USART2, USART_FLAG_TC);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART2_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART2_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

u32 count = 0;
u8 usart2_dma_buf[200];
void Usart2_DMA_Config() {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  // DMA1时钟使能

    DMA_DeInit(DMA1_Stream6);  //查STM32参考资料

    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {
    }

    /*查询芯片手册可知USART2的MDA是DMA1的数据流6,通道4*/
    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //选择通道4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;  // DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr =
        (uint32_t)usart2_dma_buf;  // DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //存储器到外设模式
    DMA_InitStructure.DMA_BufferSize =
        0;  //数据不确定,暂时设置为0,正式传输时再修改
    DMA_InitStructure.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize =
        DMA_PeripheralDataSize_Byte;  //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize =
        DMA_MemoryDataSize_Byte;                   //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst =
        DMA_MemoryBurst_Single;  //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst =
        DMA_PeripheralBurst_Single;              //外设突发单次传输
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);  //初始化DMA Stream

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  //开启USART2的DMA传输
}

int err;
void DMA_BUF_Count_Init() {
    static u8 IsOne = 1;
    if (!IsOne) {
        if (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) != SET) {
            err = 1;
        }
        while (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) != SET)
            ;
        DMA_ClearFlag(DMA1_Stream6,
                      DMA_FLAG_TCIF6 | DMA_FLAG_FEIF6 | DMA_FLAG_HTIF6);
    }
    IsOne = 0;
    count = 0;
}

void Usart2_DMA_Send_Start() {
    uint16_t ndtr;

    usart2_dma_buf[count] = '\0';

    ndtr = strlen((const char *)usart2_dma_buf);

    DMA_Cmd(DMA1_Stream6, DISABLE);  //关闭DMA传输

    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {
    }  //确保DMA可以被设置

    DMA1_Stream6->M0AR = (uint32_t)usart2_dma_buf;

    DMA_SetCurrDataCounter(DMA1_Stream6, count);  //数据传输量

    DMA_Cmd(DMA1_Stream6, ENABLE);  //开启DMA传输
}

int16_t RxDataCtr = 0;
u8 *RxDataPtr;
u8 *RxDataPtrStar;
//尚未增加上位机通信协议
void _USART2_IRQHandler(void)  //串口1中断服务程序
{
    u8 Res;
    OS_ERR err;
    static u8 Usart_Flag = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) !=
        RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
        Res = USART_ReceiveData(USART2);  //(USART1->DR);
                                          ////读取接收到的数据

        if (Res == '<') {
            RxDataCtr = 1;
            RxDataPtr = (u8 *)OSMemGet((OS_MEM *)&UsartMemory, (OS_ERR *)&err);
            RxDataPtrStar = RxDataPtr;
            *RxDataPtr = Res;
            RxDataPtr++;
        } else if (Res == '>') {
            *RxDataPtr = Res;
            RxDataPtr++;
            RxDataCtr++;

            OSTaskQPost((OS_TCB *)&TaskReceDataTCB, (void *)RxDataPtrStar,
                        (OS_MSG_SIZE)RxDataCtr, (OS_OPT)OS_OPT_POST_FIFO,
                        (OS_ERR *)&err);

            RxDataPtr = NULL;
            RxDataCtr = 0;

        } else {
            *RxDataPtr = Res;
            RxDataPtr++;
            RxDataCtr++;
        }
    }
}

void USART2_IRQHandler(void) {
    OSIntEnter();
    _USART2_IRQHandler();
    OSIntExit();
}
