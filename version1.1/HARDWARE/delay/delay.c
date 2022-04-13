#include "stm32l1xx.h"
#include "delay.h"

static unsigned char  fac_us=0;										      //us延时倍乘数			   
static unsigned short fac_ms=0;										      //ms延时倍乘数,在ucos下,代表每个节拍的ms数

void delay_init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;									      //为系统时钟的1/8  

	fac_ms=(unsigned short)fac_us*1000;							      //非OS下,代表每个ms需要的systick时钟数   
}	

void delay_us(unsigned long nus)
{		
	unsigned long temp;	    	 
	SysTick->LOAD=nus*fac_us; 											     //时间加载	  		 
	SysTick->VAL=0x00;        											     //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;				     //开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));						    //等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;				    //关闭计数器
	SysTick->VAL =0X00;      					 							    //清空计数器	 
}

void delay_ms(unsigned short nms)
{	 		  	  
	unsigned long temp;		   
	SysTick->LOAD=(unsigned long)nms*fac_ms;				    //时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							                    //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		            //等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	          //关闭计数器
	SysTick->VAL =0X00;       					                //清空计数器	  	    
}

