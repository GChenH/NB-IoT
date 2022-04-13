#include "process_task.h"

extern unsigned char usart1_buf[5];
extern unsigned int usart1_cnt;

//MCU普通模式硬件初始化
void bsp_init(void)
{ 
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	  uart2_init(9600);
		  uart1_init(115200);
	  delay_ms(10);
        
		uart2_init(9600);	//串口
	  delay_ms(10);
    //led_init();                                    //led
    spi1_init();
	  //delay_ms(2);
    exti_init();
    //delay_ms(2);
    SPI1_SetSpeed(SPI_BaudRatePrescaler_4);          //spi1提速为4MHz
	  delay_ms(10);
	  fm17550_hard_reset(); 
	  delay_ms(10);	

	  printf("clk=%02x\r\n", RCC_GetSYSCLKSource());   //0X0C-PLL
	  printf("CRGR=0x%08x\r\n", RCC->CFGR);            //0X91000F
}

//初始化LPCD
void lpcd_func(void)
{
	  //LPCD功能测试
	  unsigned char calibraflag;
    lpcd_param_init();                             //lpcd初始化参数
    lpcd_reg_init();                               //lpcd寄存器初始化
    lpcd_init_calibra(&calibraflag);               //lpcd初始化调校

    if(calibraflag == True)
    {
        printf("Calibrag success\r\n\r\n");
    }
    else
    {
        printf("Calibrag fail\r\n");
//				while(calibraflag == False)
//				{
//					lpcd_init_calibra(&calibraflag);
//					if(calibraflag == True)
//					{
//						printf("Calibrag success\r\n\r\n");
//						break;
//					}
//					
//				}
    }

    if(arm_write_ext_reg(0x03, 0x20) == OK)//bit5 HPDEn置1
    {
        if(fm17550_hardpowerdown(1) == IN_HPD)//1进入，0退出
        {
            printf("2020-2-11Enter LPCD mode Success!!\r\n");
					  //into_stop_mode_init();
        }
        else
        {
            printf("Enter LPCD mode Fail!!\r\n");
        }
    }  
}
//进入stop模式MCU初始化
void into_stop_mode_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;      //所有引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;     //模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    //GPIO_Init(GPIOA, &GPIO_InitStructure);	  
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
	  delay_init();
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	  uart1_init(115200); 
	  spi1_init();
	  //delay_ms(2);
    exti_init();
	  //delay_ms(2);
	  SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
		//fm17550_hard_reset();                         
		delay_ms(20);
		//delay_ms(2);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                //失能NPD时钟后，需要重新配置，否则处于闲空状态，IO功耗较大，这步必须做
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);		
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10; 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	  SPI_Cmd(SPI1, DISABLE);
		USART_DeInit(USART1);
		USART_Cmd(USART1, DISABLE);

    //uart1_init(115200);
    
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);

	  EXTI_ClearITPendingBit(EXTI_Line1);
		//PWR_FastWakeUpCmd(DISABLE);
		PWR_UltraLowPowerCmd(ENABLE);
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

void gpio_a2(void)
{
	 GPIO_InitTypeDef gpio_struct;
	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
   gpio_struct.GPIO_Mode = GPIO_Mode_OUT;                       
	 gpio_struct.GPIO_Pin = GPIO_Pin_2;
	 gpio_struct.GPIO_OType = GPIO_OType_PP;
	 gpio_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 gpio_struct.GPIO_Speed = GPIO_Speed_40MHz;
	 GPIO_Init(GPIOA, &gpio_struct);
}
//fm17550初始化
void fm17550_init(void)
{
    fm17550_hard_reset();                         
    delay_ms(20);
}
//任务处理函数
void process_task(void)
{
	  unsigned char tx_buf[2] = {0};
		unsigned char rx_buf[2] = {0};
		unsigned char rec_fifo_len = 0;
    switch(usart1_buf[0])
    {
        case 0x01:
        {
            usart1_cnt = 0;
            usart1_buf[0] = NULL;

            printf("abcd\r\n");
        }
        break;

        case 0x02:
        {
            usart1_cnt = 0;
            usart1_buf[0] = NULL;

            clear_fifo();     
            type_A_request(); 
        }break;
				
				case 0x03:
				{
					 usart1_cnt = 0;
           usart1_buf[0] = NULL;
					
				   spi1_hard_write_reg(waterlevel_reg, 0x05);
					 delay_ms(100);
					 printf("wt=%02x\r\n", spi1_hard_read_reg(waterlevel_reg));
				}break;
				
			  case 0x04:
				{
					 usart1_cnt = 0;
           usart1_buf[0] = NULL;
					  
					 tx_buf[0] = 0x55;
				   spi1_write_fifo(1, tx_buf);
					 delay_ms(500);
				   rec_fifo_len = spi1_hard_read_reg(fifolevel_reg);
           printf("rec_len=%02x\r\n", rec_fifo_len);
					 spi1_read_fifo(1, rx_buf);
					 printf("buf0=%02x\r\n", rx_buf[0]);
				}break;
				
				case 0x05:
				{
					 usart1_cnt = 0;
           usart1_buf[0] = NULL;	

           arm_write_ext_reg(JREG_LPCD_T1CFG, 0x03);	
           printf("t1=%02x\r\n", arm_read_ext_reg(JREG_LPCD_T1CFG));					
				}break;
				
				case 0x06:
				{
				   usart1_cnt = 0;
           usart1_buf[0] = NULL;
					
					 clear_fifo();     
           type_A_request();
				}break;

        default:
            break;
    }
}

