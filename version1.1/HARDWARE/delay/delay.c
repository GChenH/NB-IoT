#include "stm32l1xx.h"
#include "delay.h"

static unsigned char  fac_us=0;										      //us��ʱ������			   
static unsigned short fac_ms=0;										      //ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

void delay_init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;									      //Ϊϵͳʱ�ӵ�1/8  

	fac_ms=(unsigned short)fac_us*1000;							      //��OS��,����ÿ��ms��Ҫ��systickʱ����   
}	

void delay_us(unsigned long nus)
{		
	unsigned long temp;	    	 
	SysTick->LOAD=nus*fac_us; 											     //ʱ�����	  		 
	SysTick->VAL=0x00;        											     //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;				     //��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));						    //�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;				    //�رռ�����
	SysTick->VAL =0X00;      					 							    //��ռ�����	 
}

void delay_ms(unsigned short nms)
{	 		  	  
	unsigned long temp;		   
	SysTick->LOAD=(unsigned long)nms*fac_ms;				    //ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							                    //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		            //�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	          //�رռ�����
	SysTick->VAL =0X00;       					                //��ռ�����	  	    
}

