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
//��׼����Ҫ��֧�ֺ���
struct __FILE {
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
_sys_exit(int x) { x = x; }
//�ض���fputc����
int fputc(int ch, FILE *f) {
    while ((USART2->SR & 0X40) == 0)
        ;  //ѭ������,ֱ���������
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

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  // DMA1ʱ��ʹ��

    DMA_DeInit(DMA1_Stream6);  //��STM32�ο�����

    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {
    }

    /*��ѯоƬ�ֲ��֪USART2��MDA��DMA1��������6,ͨ��4*/
    /* ���� DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //ѡ��ͨ��4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;  // DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr =
        (uint32_t)usart2_dma_buf;  // DMA �洢��0��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //�洢��������ģʽ
    DMA_InitStructure.DMA_BufferSize =
        0;  //���ݲ�ȷ��,��ʱ����Ϊ0,��ʽ����ʱ���޸�
    DMA_InitStructure.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  //���������ģʽ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�洢������ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize =
        DMA_PeripheralDataSize_Byte;  //�������ݳ���:8λ
    DMA_InitStructure.DMA_MemoryDataSize =
        DMA_MemoryDataSize_Byte;                   //�洢�����ݳ���:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // ʹ����ͨģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst =
        DMA_MemoryBurst_Single;  //�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst =
        DMA_PeripheralBurst_Single;              //����ͻ�����δ���
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);  //��ʼ��DMA Stream

    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  //����USART2��DMA����
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

    DMA_Cmd(DMA1_Stream6, DISABLE);  //�ر�DMA����

    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {
    }  //ȷ��DMA���Ա�����

    DMA1_Stream6->M0AR = (uint32_t)usart2_dma_buf;

    DMA_SetCurrDataCounter(DMA1_Stream6, count);  //���ݴ�����

    DMA_Cmd(DMA1_Stream6, ENABLE);  //����DMA����
}

int16_t RxDataCtr = 0;
u8 *RxDataPtr;
u8 *RxDataPtrStar;
//��δ������λ��ͨ��Э��
void _USART2_IRQHandler(void)  //����1�жϷ������
{
    u8 Res;
    OS_ERR err;
    static u8 Usart_Flag = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) !=
        RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
    {
        Res = USART_ReceiveData(USART2);  //(USART1->DR);
                                          ////��ȡ���յ�������

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
