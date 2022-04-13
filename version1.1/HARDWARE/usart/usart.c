#include "usart.h"
#include "sys.h"


#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 





//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);                                                     //循环发送,直到发送完毕   
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
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);                              //使能GPIOA时钟  PA9--TX,PA10--RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	                         //使能USART1时钟
  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);                         //端口复用，注意：这两配置不能少
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	                                   //输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                   //推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;           				               //没上下拉
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //USART1_RX	  GPIOA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                 											 //浮空输入
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;	
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03; 											 //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		                           //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                             //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                                                 //根据指定的参数初始化VIC寄存器
  
  //USART 初始化设置
	USART_InitStructure.USART_BaudRate = baund;                                      //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                              //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                 //收发模式

  USART_Init(USART1, &USART_InitStructure);                                        //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                   //开启串口接受中断
  USART_Cmd(USART1, ENABLE);                                                       //使能串口1 
}


void uart2_init(unsigned int bound)
{

    //GPIO端口设置
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
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Usart3 NVIC 配置

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    //USART 初始化设置

    USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART2, &USART_InitStructure); //初始化串口
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART2, ENABLE);                    //使能串口

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}




UART_BUF buf_uart2; 

void Uart2_SendStr(char*SendBuf)//串口1打印数据代码
{
    while(*SendBuf)
    {
        while((USART2->SR&0X40)==0)
        {
        }//等待发送完成
        USART2->DR = (unsigned int) *SendBuf; 
        SendBuf++;
    }
}


void nbiot_receive_process_event(unsigned char ch )     //串口2给nbiot用
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

void USART2_IRQHandler(void)                            //串口2接收函数
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

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)                            //接收中断
	{
		usart1_buf[usart1_cnt++] = USART1->DR;
		//printf("999\r\n");
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
	}
} 	



