#include "lock.h"

void LOCK_IO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); 	//使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 			//PB0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;		 	//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 			//根据设定参数初始化GPIOB0
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);						 						//输出高电平
	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE); 	//使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 			//PB0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;		 	//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 			//根据设定参数初始化GPIOB0
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);			
}

