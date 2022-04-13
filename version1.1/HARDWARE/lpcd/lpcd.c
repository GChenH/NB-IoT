#include "process_task.h"

//lpcd配置
unsigned char T3ClkDivK ;      //T3 阶段工作时钟的分频比设置
unsigned char LpcdBiasCurrent ;//lpcd基准电流配置
unsigned char LpcdGainReduce;  //放大器衰减倍数
unsigned char LpcdGainAmplify; //放大器增益倍数
unsigned char LpcdADCRefernce; //ADC参考电平配置bit5-bit0

unsigned char Timer1Cfg;//4bit //配置T1阶段时长  T1_time = (T1Cfg+2)*100ms
unsigned char Timer2Cfg;//5bit //配置T2阶段时长  T2_time = （T2Cfg+2） * 100us
unsigned char Timer3Cfg;//5bit //配置T3阶段时长  T3_time = （T3Cfg-1） * 4.7us

unsigned char ADCResultFullScale;	//记录每次卡侦测的结果 LpcdADCResult_H(2bit)+LpcdADCResult_L(6bit),如果该测量值结果>侦测上阈值或<侦测下阈值，则认为有卡片进入射频场//T3下满幅ADCresult信息
unsigned char ADCResultThreshold;	//检测幅度，设置成相对值
unsigned char LpcdThreshold_L;		//LPCD幅度低阈值
unsigned char LpcdThreshold_H;		//LPCD幅度高阈值
unsigned char ADCResultCenter;		//LPCD幅度中心点
unsigned char LpcdADCResult[10];  //LPCD幅度信息，用于误触发判断

//lpcd参数初始化
unsigned char lpcd_param_init(void)
{
        LpcdBiasCurrent = LPCD_BIAS_CURRENT ; //lpcdbiascurrent 寄存器0F/05h
				LpcdGainReduce = 0x3;			//1x //增益衰减
				LpcdGainAmplify = 0x0;		//1x //增益放大
				LpcdADCRefernce = 0;      //ADC参考电平配置
	
				Timer1Cfg = TIMER1_CFG;	  //T1时长//取值范围是0x1~0xf    //3
				Timer2Cfg = TIMER2_CFG;   //T2时长//取值范围是0x2~0x1f   //13
				Timer3Cfg = TIMER3_CFG;   //T3时长//取值范围是0x2~0x1f   //11 
				
				if (Timer3Cfg > 0xF) 
				{
					 T3ClkDivK = 2;			    //T3阶段工作时钟的分频比设置16分频
					 ADCResultFullScale =  ((Timer3Cfg - 1) << 3);  //80
					 ADCResultCenter = (ADCResultFullScale >> 1);   //40
					 ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);//检测幅度，设置成相对值  5
				}
				else if(Timer3Cfg > 0x7) //
				{
					 T3ClkDivK = 1;			   //T3阶段工作时钟为8分频
					 //T3ClkDivK = 0;      //4分频
					 ADCResultFullScale =  ((Timer3Cfg - 1) << 4);  //160
					 ADCResultCenter = (ADCResultFullScale >> 1);  //80
					 ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO); //10
				}
				else 
				{
					T3ClkDivK = 0;			   //T3阶段工作时钟为4分频
					ADCResultFullScale =  ((Timer3Cfg - 1)<<5);
					ADCResultCenter = (ADCResultFullScale >>1);
					ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);
				}

				LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;  //80+10=90
				LpcdThreshold_L= ADCResultCenter - ADCResultThreshold;   //80-10=70

				return OK;
}

//lpcd寄存器初始化
unsigned char lpcd_reg_init()
{
			spi1_hard_write_reg(comiend_reg, 0x80);//IRQInv位置1，引脚IRQ的信号被置为寄存器Status1Reg的IRQ位的相反
			spi1_hard_write_reg(divien_reg, 0x00);//IRQPushPull位置1，不需上拉电阻(CMOS)，置0，需要上拉电阻(OC)，根据硬件电路决定该位配置
			 
			//初始化配置LPCDCTRL1控制寄存器	
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);           //扩展寄存器lpcdctrl1配置//复位LPCD	  
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);           //扩展寄存器配置//复位放开LPCD
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_EN);             //使能LPCD
			arm_write_ext_reg(JREG_LPCD_CTRL1, (LPCD_IE<<5)+JBIT_LPCD_IE);                  //配置LPCD中断，配置LPCD中断寄存器状态反映到IRQ引脚
			arm_write_ext_reg(JREG_LPCD_CTRL1, (LPCD_AUTO_DETECT_TIMES<<5)+JBIT_LPCD_CMP_1);//配置进场检测此时，一次检测到卡有效
			 
			//初始化配置LPCDCTRL2扩展寄存器
			arm_write_ext_reg(JREG_LPCD_CTRL2, ((LPCD_TX2RFEN<<4)+(LPCD_CWN<<3)+LPCD_CWP));//P管驱动能力从0到7依次增大，这里选3
			 
			//初始化配置比LPCDTRL3
			arm_write_ext_reg(JREG_LPCD_CTRL3, LPCD_MODE<<3);
			
			//LPCDT1Cfg 0X07寄存器
			arm_write_ext_reg(JREG_LPCD_T1CFG, (T3ClkDivK<<4)+Timer1Cfg);
			//LPCDT2Cfg 0X08寄存器
			arm_write_ext_reg(JREG_LPCD_T2CFG, Timer2Cfg);
			//LPCDT3Cfg 0x09寄存器
			arm_write_ext_reg(JREG_LPCD_T3CFG, Timer3Cfg);
			//LpcdVmidBdCfg 0x0a
			arm_write_ext_reg(JREG_LPCD_VMIDBD_CFG, VMID_BG_CFG);//不建议用户修改
			//LpcdAutoWupCfg   0x0b
			arm_write_ext_reg(JREG_LPCD_AUTO_WUP_CFG, (AUTO_WUP_EN << 3) + AUTO_WUP_CFG);//设置自动唤醒时间
			//LpcdthresholdMin_L 0x0e
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MIN_L, (LpcdThreshold_L & 0x3F));
			//LpcdThresholdMin_H 0x0f
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MIN_H, (LpcdThreshold_L >> 6));//设置卡检测下阈值
			//LpcdThresholdMax_L 0x10
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MAX_L, (LpcdThreshold_H & 0x3F));
			//LpcdThresholdMax_H 0x11
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MAX_H, (LpcdThreshold_H >> 6));//设置检卡上阈值
			//LpcdAutoWupCfg 0x0b
			arm_write_ext_reg(JREG_LPCD_AUTO_WUP_CFG, (AUTO_WUP_EN << 3)+ AUTO_WUP_CFG); //再次设置自动唤醒时间
			
			return OK;
}
//初始化调校
unsigned char lpcd_init_calibra(unsigned char *calibraflag)
{
      unsigned char  ret;
			unsigned char  ADCResult;						//LPCD幅度数值
			unsigned char  GainCalibraFlag;			//增益校准标志
			unsigned char  ADCResult_Pre;				//前一次幅度数值
	
			arm_write_ext_reg(JREG_LPCD_CTRL4, ((LpcdGainAmplify << 2) + LpcdGainReduce)); //配置增益
			arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT, ((LpcdADCRefernce&0x40)>>1) + LpcdBiasCurrent&0x7); //配置偏置电流和参考电压
			arm_write_ext_reg(JREG_LPCD_MISC, BFL_JBIT_CALIB_VMID_EN); 	// CalibVmidEnable//增加CalibVmidEn使能
			arm_write_ext_reg(JREG_LPCD_T1CFG, (T3ClkDivK<<4) + Timer1Cfg); //Timer1Cfg
			arm_write_ext_reg(JREG_LPCD_T2CFG, Timer2Cfg);					//Timer2Cfg
			arm_write_ext_reg(JREG_LPCD_T3CFG, Timer3Cfg); 					//Timer3Cfg
	
			LpcdADCRefernce = ADC_REFERNCE_MIN;											//参考电压设置为最小值
	
			arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT, ((LpcdADCRefernce&0x40)>>1) + LpcdBiasCurrent&0x7);
			arm_write_ext_reg(JREG_LPCD_ADC_REFERECE, LpcdADCRefernce&0x3F);
			//2020-2-11
			ret = calibra_read_adcresult(&ADCResult);	//读取幅度数值
	    printf("1--adc=%02x\r\n", ADCResult);     //92
			GainCalibraFlag = True;										//缺省增益校准完成

			if(ADCResult < ADCResultCenter) 	        //幅度数值过小，需要减小增益
		  {
				    //printf("66\r\n");
						GainCalibraFlag = False;
						while(1)
						{
						
							if(LpcdGainReduce == 0) 	        //增益已经为最小值，校准失败
							{
								GainCalibraFlag = False;
								break;
							}
						
							LpcdGainReduce --; 	             //增益减小，继续衰减						 
						
							arm_write_ext_reg(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));	//设置增益
							
							
							ret = calibra_read_adcresult(&ADCResult);	//读取幅度数值
										
							
							if (ADCResult > ADCResultCenter)	//校准完成，把中心点移到中心点右侧
							{
								GainCalibraFlag = True;
								break;
							}
						}
			}
			else
			{						
				LpcdADCRefernce = ADC_REFERNCE_MAX;	//参考电压设置为最大值--127
				arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);//0
				arm_write_ext_reg(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);//63
				
				ret = calibra_read_adcresult(&ADCResult);//读取幅度数值
				printf("2--adc=%02x\r\n", ADCResult); 		
				GainCalibraFlag = True;                  //缺省增益校准完成
				//printf("77\r\n");
				
				if (ADCResult > ADCResultCenter) //判断是否幅度太小，如果太小lpcd_gain放大
				{
					GainCalibraFlag = False;  
					while(1)
					{
					  //如果当前已经是最大增益，调校失败
						if (LpcdGainAmplify == 0x7)
						{
							  GainCalibraFlag = False;//增益已经为最大值，校准失败
							  break;
						}
						else
						{
							  LpcdGainAmplify++; //增益加大
						}
						
						arm_write_ext_reg(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce)); //设置增益
									
						ret = calibra_read_adcresult(&ADCResult);	//读取幅度数值
						printf("Amplify=%02x\r\n", LpcdGainAmplify);	
  					printf("3--adc=%02x\r\n", ADCResult); 	
					  //如果调校成功，把中心点移到中心点左侧
						if(ADCResult < ADCResultCenter)
						{
								GainCalibraFlag = True;	 	//校准完成
								break;
						}
					}

				}
			}
       
			//如果增益调校失败，则失败
			if(GainCalibraFlag == False)
			{		
				(*calibraflag) = False;
				return ADCResult;	   //校准失败，返回幅度数值
			}
      
			//扫描参考电压值，找到合适的穿越中心点的配置
			(*calibraflag) = False;	//有疑问
			
			//调校过程ADC的参考电压设定
			ADCResult_Pre = ADCResult; //保存幅度数值
			
       
      printf("111\r\n");			
      //扫描充电电容，通过此循环可以确定最终的参考电压和ADC中心比较值			
			for(LpcdADCRefernce = ADC_REFERNCE_MIN; LpcdADCRefernce < ADC_REFERNCE_MAX; LpcdADCRefernce++)
			{
				//配置参考电压值
				printf("adcrf=%02x\r\n",LpcdADCRefernce);
				arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
				
				arm_write_ext_reg(JREG_LPCD_ADC_REFERECE, LpcdADCRefernce&0x3F);
				
				ADCResult_Pre = ADCResult;	//保存幅度数值，备份幅度信息
			  
				ret = calibra_read_adcresult(&ADCResult);	//读取幅度数值
				//printf("Adc[%d]=%02x\r\n",LpcdADCRefernce, ADCResult);
				//如果幅度开始比中心值小，因为开始比中心值大
				if(ADCResult < ADCResultCenter)
				{
//					printf("adc_pe=%02x\r\n", ADCResult_Pre);
//					printf("11Adc=%02x\r\n",ADCResult);
//					printf("adccenter=%02x\r\n", ADCResultCenter);
					//调教成功
					(*calibraflag) = True;
				  
					if((ADCResultCenter-ADCResult) < (ADCResult_Pre-ADCResultCenter)) 	//判断当前幅度数值与前一次幅度数值，哪个更接近中心点
					{
							ADCResultCenter = ADCResult;	//当前幅度数值作为参考中心点，因为比前一次更靠近设置的中心点
//					    printf("hh\r\n");
					}
					else
					{
						//printf("kk\r\n");
						ADCResultCenter = ADCResult_Pre; //前一次幅度数值作为参考中心点
					
						LpcdADCRefernce--;	             //减小参考电压
					
						arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
						
						arm_write_ext_reg(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);//一直在改变			
					}
					
					//调整阀值//设置灵敏度检测
					LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;
					LpcdThreshold_L = ADCResultCenter - ADCResultThreshold;
					printf("ADCResultCenter=%02x\r\n", ADCResultCenter);
					printf("ADCResultThreshold=%02x\r\n", ADCResultThreshold);
					printf("lpcdH=%02x\r\n", LpcdThreshold_H);
					printf("lpcdL=%02x\r\n", LpcdThreshold_L);
					//-----------------------------------------------------------------
					//LpcdThreshold_L1
					//-----------------------------------------------------------------
					arm_write_ext_reg(JREG_LPCD_THRESHOLD_MIN_L,(LpcdThreshold_L&0x3F));
					//-----------------------------------------------------------------
					//LpcdThreshold_L2
					//-----------------------------------------------------------------
					arm_write_ext_reg(JREG_LPCD_THRESHOLD_MIN_H,(LpcdThreshold_L>>6));
					//-----------------------------------------------------------------
					//LpcdThreshold_H1
					//-----------------------------------------------------------------
					arm_write_ext_reg(JREG_LPCD_THRESHOLD_MAX_L,(LpcdThreshold_H& 0x3F));
					//-----------------------------------------------------------------
					//LpcdThreshold_H2
					//-----------------------------------------------------------------
					arm_write_ext_reg(JREG_LPCD_THRESHOLD_MAX_H,(LpcdThreshold_H>>6));
					break;
				}
				
			}
			printf("222\r\n");
			
			clear_ext_bitmask (JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN);             //关闭CalibVmidEn
			if (GainCalibraFlag == False)
			{		
				(*calibraflag) = False;
				return ADCResult;                                                   //校准失败，返回幅度数值
			}
			return OK;
}

//调校并读取LPCD幅度数值
unsigned char calibra_read_adcresult(unsigned char *adcresult)
{
      unsigned char ret;
			//使能校准
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN);
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_CALIBRA_EN);
			
			//等待校准中断
			ret = wait_for_lpcdirq(JBIT_CALIB_IRQ);	
			
	    //关闭调校模式
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN);
	
	   //读取幅度数值信息
			read_lpcd_adcresult(adcresult);
			return OK;
}

//等待校准中断
unsigned char wait_for_lpcdirq(unsigned char irqtype)
{
      unsigned char ExtRegData;
	    unsigned char TimeOutCount;
	
			TimeOutCount = 0;
			ExtRegData = arm_read_ext_reg(JREG_LPCD_IRQ);// 表示irqtype=JBIT_CALIB_IRQ-->calib 调校模式完成一次卡侦测
      
			while ((ExtRegData & irqtype) != irqtype)
			{
				ExtRegData = arm_read_ext_reg(JREG_LPCD_IRQ);				
				delay_ms(1);//延时10ms
				TimeOutCount++;
				
				if  (TimeOutCount > IRQ_TIMEOUT)  
				{
					return ERROR; // 超时错误
				}
			}
			printf("etd=%02x\r\n", ExtRegData);
			return OK;
}

//读取幅度数值
unsigned char read_lpcd_adcresult(unsigned char *adcresult)
{
        unsigned char  ExtRegData;
				unsigned char  ADCResultTemp;

				*adcresult = 0;
				ExtRegData = arm_read_ext_reg(JREG_LPCD_ADC_RESULT_H);//读取高位字节（BIT7，BIT6）
				ADCResultTemp = ((ExtRegData & 0x3) << 6);
				ExtRegData = arm_read_ext_reg(JREG_LPCD_ADC_RESULT_L);//读取低位字节（BIT5~BIT0）
				ADCResultTemp += (ExtRegData & 0x3F);
				*adcresult = ADCResultTemp;

				//RESET LPCD
				arm_write_ext_reg(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);
				//LPCD ENABLE
				arm_write_ext_reg(JREG_LPCD_CTRL1,JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);
				 
				return OK;
}

void lpcd_auto_wakeup_irq_handler(void)
{
        unsigned char CalibraFlag;
				delay_ms(20); //20ms
	       
				lpcd_param_init();						    //LPCD参数初始化
				lpcd_reg_init();       				    //LPCD寄存器初始化
			  //LpcdAuxSelect(ON);						  //开启AUX观测通道
				lpcd_init_calibra(&CalibraFlag);  //LPCD初始化调教

				if (CalibraFlag == True)				  //如果调教成功，则亮灯
				{
					fm17500_hard_powerdown(1);			//进入LPCD模式
				}
				else
				{
				   printf("Calibraflag fail ***\r\n");
				}
				//delay_ms(2); 
				//return;
}

/*********************************************/
//函数名：	    fm17500_hard_powerdown
//功能：	      硬件低功耗操作
//输入参数：	  mode=1 进入低功耗模式, mode=0 退出低功耗模式
//返回值：	    OK，进入低功耗模式；
//				      ERROR，退出低功耗模式；
/*********************************************/
unsigned char fm17500_hard_powerdown(unsigned char mode)
{
     if(mode == 0)
		 {
		     NPD_RST = 1;
			   delay_us(20);
			   return ERROR;
		 }
		 
		 else if(mode == 1)
		 {
		    NPD_RST = 0;
			  delay_us(20);
		 }
		 return OK; //进入低功耗模式
}

void lpcd_cardin_irqhandler(void)
{
//      delay_ms(20); 
//			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN); 	//复位LPCD寄存器
//			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);   //复位放开LPCD寄存器
//	     
	    
			//因为每次唤醒，测试寄存器会被复位
		  //LpcdAuxSelect(ON);					
			//需要延时，否则有可能最后一个寄存器写不进

//			LED0 = 1;
//			delay_ms(1000);
//			LED0 = 0;
//	    LED1   = 1;
			//printf("Card in\r\n");
	    //printf("t33=%02x\r\n",arm_read_ext_reg(0x09));
	   // printf("irq=%02x\r\n",arm_read_ext_reg(0x12));
			//--------------------------------------------------
			
			//--------------------------------------------------
			//用户在此添加判断卡片进场之后的处理，比如request等
			//也可以根据卡片是否真实进场，重新调用自动调校程序
			//--------------------------------------------------
	    type_A_request();
			   
			//fm17500_hard_powerdown(1);	//继续进入LPCD模式
			//delay_ms(5);
		  //return;
}
