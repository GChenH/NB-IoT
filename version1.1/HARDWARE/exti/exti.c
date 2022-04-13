#include "process_task.h"
#include "stm32l1xx_syscfg.h"

unsigned char flag = 0;
unsigned char lpcd_irq = 0;

//IRQ引脚初始化PA1
void exit_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	//RFID
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;         //2020-9-20
    //GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_10MHz;
    //GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	//sensor
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//外部中断初始化
void exti_init(void)
{

    //exit_gpio_init();
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //EXTI_DeInit();
    //exit_gpio_init();
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;              //内部配置上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		//sensor
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;              //内部配置上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
		
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
//中断服务函数
void EXTI1_IRQHandler(void)
{

	  //delay_ms(10);
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
		{
		   EXTI_ClearITPendingBit(EXTI_Line1);
        SystemInit();
			  //RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
        //bsp_init();
			  delay_ms(20);
			  uart1_init(115200);
			  //printf("sourece = %02x\r\n", RCC_GetSYSCLKSource());

        //LED0 = ~LED0;
			  //flag = 1;
        
		}
}
*/


void EXTI1_IRQHandler(void)
{
//    unsigned char lpcd_irq;
    unsigned char adctmp;
     
	  //delay_ms(10);
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {		
        SystemInit();
        delay_ms(3);
			  uart1_init(115200);
				uart2_init(9600);
			  delay_ms(3);
			  //spi1_init();
			  SPI_Cmd(SPI1, ENABLE);
			  delay_ms(2);
			  //printf("66\r\n");
        fm17500_hard_powerdown(0);    //NPD = 1;
        delay_ms(3);																											
        lpcd_irq = arm_read_ext_reg(JREG_LPCD_IRQ);
				 
			  //printf("irq=%02x\r\n", lpcd_irq);
        //delay_ms(10);

//        if((lpcd_irq & JBIT_AUTO_WUP_IRQ) == JBIT_AUTO_WUP_IRQ)//定时唤醒，自动调校
//        {
//            lpcd_auto_wakeup_irq_handler();
//        }

//        else if ((lpcd_irq & JBIT_CARD_IN_IRQ) == JBIT_CARD_IN_IRQ)	//卡片进场
//        {
//            lpcd_cardin_irqhandler();
//        }

//	  if(((lpcd_irq & JBIT_AUTO_WUP_IRQ) == JBIT_AUTO_WUP_IRQ) ||((lpcd_irq & JBIT_CARD_IN_IRQ) == JBIT_CARD_IN_IRQ))
//		{
//		   flag = 1;
//				else
//				{
//				
//				}
//		}
        //EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE2上的中断标志位
				EXTI_ClearITPendingBit(EXTI_Line1);
    }

}


void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0 != RESET))
	{
		SystemInit();
    delay_ms(3);
		uart2_init(9600);
		delay_ms(3);
		
		EXTI_ClearITPendingBit(EXTI_Line0);
	}

}

