#include "usart.h"
#include "sys.h"


#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 





//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);                                                     //ѭ������,ֱ���������   
    USART1->DR = (unsigned char) ch;      
	return ch;
}
#endif 




 
unsigned int USART_RX_BUF[USART_REC_LEN];
unsigned int USART_RX_STA=0;
unsigned char usart1_buf[50];
unsigned  short usart1_cnt = 0;





void uart1_init(unsigned int baund)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                              //ʹ��GPIOAʱ��  PA9--TX,PA10--RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	                         //ʹ��USART1ʱ��
  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);                         //�˿ڸ��ã�ע�⣺�������ò�����
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	                                   //���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                   //����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;           				               //û������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //USART1_RX	  GPIOA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                 											 //��������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03; 											 //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		                           //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                             //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                                                 //����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
  //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = baund;                                      //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                              //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                 //�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure);                                        //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                   //�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                                                       //ʹ�ܴ���1 
}


void uart2_init(unsigned int bound)
{

    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

    //USART2_TX -> PA2 , USART2_RX -> PA3

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Usart3 NVIC ����

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    //USART ��ʼ������

    USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART2, &USART_InitStructure); //��ʼ������
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}




UART_BUF buf_uart2; 

void Uart2_SendStr(char*SendBuf)//����1��ӡ���ݴ���
{
    while(*SendBuf)
    {
        while((USART2->SR&0X40)==0)
        {
        }//�ȴ��������
        USART2->DR = (unsigned int) *SendBuf; 
        SendBuf++;
    }
}


void nbiot_receive_process_event(unsigned char ch )     //����2��nbiot��
{
    if(buf_uart2.index >= BUFLEN)
    {
        buf_uart2.index = 0 ;
    }
    else
    {
        buf_uart2.buf[buf_uart2.index++] = ch;
    }
}

void USART2_IRQHandler(void)                            //����2���պ���
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)
    {
        nbiot_receive_process_event(USART_ReceiveData(USART2));
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }

    if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)==SET)
    {
        nbiot_receive_process_event(USART_ReceiveData(USART2));
        USART_ClearFlag(USART2,USART_FLAG_ORE);
    }
}

//����1�жϷ������
void USART1_IRQHandler(void)                	
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)                            //�����ж�
	{
		usart1_buf[usart1_cnt++] = USART1->DR;
		//printf("999\r\n");
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
	}
} 	



