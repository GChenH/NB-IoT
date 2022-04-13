#ifndef __USART_H
#define __USART_H

#include "sys.h"
#include "stdio.h"	 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern unsigned int  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern unsigned int  USART_RX_STA;         		//����״̬���	

void uart1_init(unsigned int baund);
void uart2_init(unsigned int bound);
void Uart2_SendStr(char*SendBuf);//����1��ӡ���ݴ���;

#define EN_USART1_RX 			1	
#define BUFLEN 256      //���黺���С
typedef struct _UART_BUF
{
    char buf [BUFLEN+1];               
    unsigned int index ;
}UART_BUF;

#endif

