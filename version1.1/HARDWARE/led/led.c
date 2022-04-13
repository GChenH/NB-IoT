#include "led.h"

void led_init(void)
{
	  GPIO_InitTypeDef gpio_struct;
	  
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);      //Ê¹ÄÜGPIOÊ±ÖÓ
	  gpio_struct.GPIO_Mode = GPIO_Mode_OUT;
	  gpio_struct.GPIO_OType = GPIO_OType_PP;
	  gpio_struct.GPIO_Pin = GPIO_Pin_11;
	  gpio_struct.GPIO_PuPd = GPIO_PuPd_UP;
	  gpio_struct.GPIO_Speed = GPIO_Speed_2MHz;
	
	  GPIO_Init(GPIOA, &gpio_struct);
	  GPIO_SetBits(GPIOA, GPIO_Pin_11);
}

