#include "lock.h"

void LOCK_IO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); 	//ʹ��GPIOBʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 			//PB0 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;		 	//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 			//�����趨������ʼ��GPIOB0
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);						 						//����ߵ�ƽ
	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); 	//ʹ��GPIOBʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 			//PB0 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;		 	//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 			//�����趨������ʼ��GPIOB0
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);			
}

