#ifndef __BC_28_H_
#define __BC_28_H_
#include "usart.h"
#include "stm32l1xx_it.h"
#include "delay.h"
void Clear_Buffer(void);//��ջ���	
int BC28_Init(void);
void BC28_PDPACT(void);
void BC28_ConTCP(void);
//void BC28_ConTCP(void);
void BC28_RECData(void);
void BC28_Senddata(uint8_t *len,uint8_t *data);
typedef struct
{
   uint8_t CSQ;    
   uint8_t Socketnum;   //���
   uint8_t reclen[10];   //��ȡ�����ݵĳ���
   uint8_t res;      
   uint8_t recdatalen[10];
   uint8_t recdata[100];
} BC28;
#endif
