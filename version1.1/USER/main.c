/*******************************
  *平台：stm32l151c8t6
	*功能：整板低功耗包括MCU+RFID
	*date: 2020-9-1
	*Aut : xjf
	******************************/
//1.1版本功耗在6~7 微安左右
#include "process_task.h"
#include "BC28.h"
#include "string.h"
#include "lock.h"

extern unsigned char flag;
extern unsigned char lpcd_irq;


	extern UART_BUF buf_uart2;
unsigned char card_[5]={0};
uint8_t data_return[20]={0};
char *strx1;
int errcount1=0; 	
int enter_flag = 0;



void change(uint8_t data[],char len){
	//uint8_t data_return[20]={0};
	uint8_t i=0;
	for(i=0;i<len;i++){
		data_return[2*i]  = (data[i]&0xf0)>>4;
		data_return[2*i+1]=(data[i]&0x0f);
	}
	
	for(i=0;i<2*len;i++){
		uint8_t temp=data_return[i];
		if(0<=temp && temp <=9){
			data_return[i]+=0x30;
		}
		else{
			data_return[i]+=0x41-10;
		}
	}
	//return data_return;
}





int main(void)
{
    unsigned char irq = 0;
    //////////////////////////////////////////////////////////////////
    //进入stop低功耗模式
    #if ENTER_STOP_MODE
	
    bsp_init();
    into_stop_mode_init();

    while(1)
    {
        bsp_init();
        printf("666\r\n");
        //type_A_request();
        into_stop_mode_init();
    }

    #endif
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    #if NORMAL_LOWPER_MODE
    bsp_init();                                    //MCU硬件初始化
		BC28_Init();		//init
		delay_ms(300);
    lpcd_func();         
		Clear_Buffer();		//LPCD初始化及调校
    into_stop_mode_init();
		
    while(1)
    {   
				
				delay_init();
				NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
			  uart1_init(115200);
				delay_ms(10);
				uart2_init(9600);	//串口
				delay_ms(10);  
//				strx1=NULL;
//				BC28_ConTCP();
			  printf("irq=%02x\r\n", lpcd_irq);
			  arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);
        arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);
        //arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR + JBIT_LPCD_IE);
				
				if((lpcd_irq & JBIT_CARD_IN_IRQ) == JBIT_CARD_IN_IRQ)	//卡片进场
        {
					  lpcd_irq = 0;				 					
            type_A_request();					
					  delay_ms(300);

					if(card_[0])
						{	
							delay_ms(100);
							Clear_Buffer();	
							BC28_ConTCP();
//					change(card_,3);
//					printf("%s\r\n",data_return);
//					memset(data_return,0,20);
					BC28_Senddata("11","7B01000B3030303030317B");	
					BC28_RECData();
					errcount1=0;
					//lock_count=0;
					Clear_Buffer();
					BC28_RECData();
					Clear_Buffer();
				  BC28_Senddata("11","7B06000B3030303030317B");	
								
					while(enter_flag == 0)	
					{
						
//							printf("000\r\n");
							delay_ms(300);
							errcount1++;
							BC28_RECData();					
							strx1=strstr((const char*)buf_uart2.buf,(const char*)"7B860011317B");//返回OK		
							if(strx1!=NULL){
								printf("********************getget**********************\r\n");
								printf("********************getget**********************\r\n");
								//you can unlock here	
								LOCK =1;
								printf("********************getget**********************\r\n");
								printf("********************getget**********************\r\n");
								
								if(LOCK == 1)
									{
										enter_flag=1;
										delay_ms(2000);
										LOCK = 0;
//										BC28_Senddata("5","1111111111");
									}
								break;
							}
													
							if(errcount1>10)     //防止死循环
							{
									errcount1 = 0;
									break;
							}
							printf("*************NONONONONONONONONONONO************\r\n");
							delay_ms(50);
					}
					Clear_Buffer();	
			}
		enter_flag=0;

		printf("**************************************************************************************");
			lpcd_func();
        }
			  else if((lpcd_irq & JBIT_AUTO_WUP_IRQ) == JBIT_AUTO_WUP_IRQ)//定时唤醒，自动调校
        {
					  lpcd_irq = 0;
            lpcd_auto_wakeup_irq_handler();
					  delay_ms(10);
					  printf("66\r\n");
						//BC28_Init();
        }

				else{}
				Uart2_SendStr("AT+NSOCL=1\r\n");//关闭socekt连接
				Uart2_SendStr("AT+NSOCL=2\r\n");//关闭socekt连接
				delay_ms(300);
				Clear_Buffer();	
				into_stop_mode_init();
				//arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET + JBIT_LPCD_IE);
    }
    #endif
		/////////////////////////////////////////////////////////////////
		#if NORMAL_MODE
		bsp_init();	
	  while(1)
		{
       process_task();                            //任务处理函数
	  }
    #endif
		/////////////////////////////////////////////////////////////////
    return 0;
}