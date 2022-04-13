#ifndef __USART_H
#define __USART_H

#include "sys.h"
#include "stdio.h"	 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern unsigned int  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern unsigned int  USART_RX_STA;         		//接收状态标记	

void uart1_init(unsigned int baund);
void uart2_init(unsigned int bound);
void Uart2_SendStr(char*SendBuf);//串口1打印数据代码;

#define EN_USART1_RX 			1	
#define BUFLEN 256      //数组缓存大小
typedef struct _UART_BUF
{
    char buf [BUFLEN+1];               
    unsigned int index ;
}UART_BUF;

#endif

