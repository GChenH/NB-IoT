#include "process_task.h"

//lpcd����
unsigned char T3ClkDivK ;      //T3 �׶ι���ʱ�ӵķ�Ƶ������
unsigned char LpcdBiasCurrent ;//lpcd��׼��������
unsigned char LpcdGainReduce;  //�Ŵ���˥������
unsigned char LpcdGainAmplify; //�Ŵ������汶��
unsigned char LpcdADCRefernce; //ADC�ο���ƽ����bit5-bit0

unsigned char Timer1Cfg;//4bit //����T1�׶�ʱ��  T1_time = (T1Cfg+2)*100ms
unsigned char Timer2Cfg;//5bit //����T2�׶�ʱ��  T2_time = ��T2Cfg+2�� * 100us
unsigned char Timer3Cfg;//5bit //����T3�׶�ʱ��  T3_time = ��T3Cfg-1�� * 4.7us

unsigned char ADCResultFullScale;	//��¼ÿ�ο����Ľ�� LpcdADCResult_H(2bit)+LpcdADCResult_L(6bit),����ò���ֵ���>�������ֵ��<�������ֵ������Ϊ�п�Ƭ������Ƶ��//T3������ADCresult��Ϣ
unsigned char ADCResultThreshold;	//�����ȣ����ó����ֵ
unsigned char LpcdThreshold_L;		//LPCD���ȵ���ֵ
unsigned char LpcdThreshold_H;		//LPCD���ȸ���ֵ
unsigned char ADCResultCenter;		//LPCD�������ĵ�
unsigned char LpcdADCResult[10];  //LPCD������Ϣ�������󴥷��ж�

//lpcd������ʼ��
unsigned char lpcd_param_init(void)
{
        LpcdBiasCurrent = LPCD_BIAS_CURRENT ; //lpcdbiascurrent �Ĵ���0F/05h
				LpcdGainReduce = 0x3;			//1x //����˥��
				LpcdGainAmplify = 0x0;		//1x //����Ŵ�
				LpcdADCRefernce = 0;      //ADC�ο���ƽ����
	
				Timer1Cfg = TIMER1_CFG;	  //T1ʱ��//ȡֵ��Χ��0x1~0xf    //3
				Timer2Cfg = TIMER2_CFG;   //T2ʱ��//ȡֵ��Χ��0x2~0x1f   //13
				Timer3Cfg = TIMER3_CFG;   //T3ʱ��//ȡֵ��Χ��0x2~0x1f   //11 
				
				if (Timer3Cfg > 0xF) 
				{
					 T3ClkDivK = 2;			    //T3�׶ι���ʱ�ӵķ�Ƶ������16��Ƶ
					 ADCResultFullScale =  ((Timer3Cfg - 1) << 3);  //80
					 ADCResultCenter = (ADCResultFullScale >> 1);   //40
					 ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);//�����ȣ����ó����ֵ  5
				}
				else if(Timer3Cfg > 0x7) //
				{
					 T3ClkDivK = 1;			   //T3�׶ι���ʱ��Ϊ8��Ƶ
					 //T3ClkDivK = 0;      //4��Ƶ
					 ADCResultFullScale =  ((Timer3Cfg - 1) << 4);  //160
					 ADCResultCenter = (ADCResultFullScale >> 1);  //80
					 ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO); //10
				}
				else 
				{
					T3ClkDivK = 0;			   //T3�׶ι���ʱ��Ϊ4��Ƶ
					ADCResultFullScale =  ((Timer3Cfg - 1)<<5);
					ADCResultCenter = (ADCResultFullScale >>1);
					ADCResultThreshold = (ADCResultFullScale >> LPCD_THRESHOLD_RATIO);
				}

				LpcdThreshold_H = ADCResultCenter + ADCResultThreshold;  //80+10=90
				LpcdThreshold_L= ADCResultCenter - ADCResultThreshold;   //80-10=70

				return OK;
}

//lpcd�Ĵ�����ʼ��
unsigned char lpcd_reg_init()
{
			spi1_hard_write_reg(comiend_reg, 0x80);//IRQInvλ��1������IRQ���źű���Ϊ�Ĵ���Status1Reg��IRQλ���෴
			spi1_hard_write_reg(divien_reg, 0x00);//IRQPushPullλ��1��������������(CMOS)����0����Ҫ��������(OC)������Ӳ����·������λ����
			 
			//��ʼ������LPCDCTRL1���ƼĴ���	
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);           //��չ�Ĵ���lpcdctrl1����//��λLPCD	  
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);           //��չ�Ĵ�������//��λ�ſ�LPCD
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_EN);             //ʹ��LPCD
			arm_write_ext_reg(JREG_LPCD_CTRL1, (LPCD_IE<<5)+JBIT_LPCD_IE);                  //����LPCD�жϣ�����LPCD�жϼĴ���״̬��ӳ��IRQ����
			arm_write_ext_reg(JREG_LPCD_CTRL1, (LPCD_AUTO_DETECT_TIMES<<5)+JBIT_LPCD_CMP_1);//���ý�������ʱ��һ�μ�⵽����Ч
			 
			//��ʼ������LPCDCTRL2��չ�Ĵ���
			arm_write_ext_reg(JREG_LPCD_CTRL2, ((LPCD_TX2RFEN<<4)+(LPCD_CWN<<3)+LPCD_CWP));//P������������0��7������������ѡ3
			 
			//��ʼ�����ñ�LPCDTRL3
			arm_write_ext_reg(JREG_LPCD_CTRL3, LPCD_MODE<<3);
			
			//LPCDT1Cfg 0X07�Ĵ���
			arm_write_ext_reg(JREG_LPCD_T1CFG, (T3ClkDivK<<4)+Timer1Cfg);
			//LPCDT2Cfg 0X08�Ĵ���
			arm_write_ext_reg(JREG_LPCD_T2CFG, Timer2Cfg);
			//LPCDT3Cfg 0x09�Ĵ���
			arm_write_ext_reg(JREG_LPCD_T3CFG, Timer3Cfg);
			//LpcdVmidBdCfg 0x0a
			arm_write_ext_reg(JREG_LPCD_VMIDBD_CFG, VMID_BG_CFG);//�������û��޸�
			//LpcdAutoWupCfg   0x0b
			arm_write_ext_reg(JREG_LPCD_AUTO_WUP_CFG, (AUTO_WUP_EN << 3) + AUTO_WUP_CFG);//�����Զ�����ʱ��
			//LpcdthresholdMin_L 0x0e
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MIN_L, (LpcdThreshold_L & 0x3F));
			//LpcdThresholdMin_H 0x0f
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MIN_H, (LpcdThreshold_L >> 6));//���ÿ��������ֵ
			//LpcdThresholdMax_L 0x10
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MAX_L, (LpcdThreshold_H & 0x3F));
			//LpcdThresholdMax_H 0x11
			arm_write_ext_reg(JREG_LPCD_THRESHOLD_MAX_H, (LpcdThreshold_H >> 6));//���ü쿨����ֵ
			//LpcdAutoWupCfg 0x0b
			arm_write_ext_reg(JREG_LPCD_AUTO_WUP_CFG, (AUTO_WUP_EN << 3)+ AUTO_WUP_CFG); //�ٴ������Զ�����ʱ��
			
			return OK;
}
//��ʼ����У
unsigned char lpcd_init_calibra(unsigned char *calibraflag)
{
      unsigned char  ret;
			unsigned char  ADCResult;						//LPCD������ֵ
			unsigned char  GainCalibraFlag;			//����У׼��־
			unsigned char  ADCResult_Pre;				//ǰһ�η�����ֵ
	
			arm_write_ext_reg(JREG_LPCD_CTRL4, ((LpcdGainAmplify << 2) + LpcdGainReduce)); //��������
			arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT, ((LpcdADCRefernce&0x40)>>1) + LpcdBiasCurrent&0x7); //����ƫ�õ����Ͳο���ѹ
			arm_write_ext_reg(JREG_LPCD_MISC, BFL_JBIT_CALIB_VMID_EN); 	// CalibVmidEnable//����CalibVmidEnʹ��
			arm_write_ext_reg(JREG_LPCD_T1CFG, (T3ClkDivK<<4) + Timer1Cfg); //Timer1Cfg
			arm_write_ext_reg(JREG_LPCD_T2CFG, Timer2Cfg);					//Timer2Cfg
			arm_write_ext_reg(JREG_LPCD_T3CFG, Timer3Cfg); 					//Timer3Cfg
	
			LpcdADCRefernce = ADC_REFERNCE_MIN;											//�ο���ѹ����Ϊ��Сֵ
	
			arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT, ((LpcdADCRefernce&0x40)>>1) + LpcdBiasCurrent&0x7);
			arm_write_ext_reg(JREG_LPCD_ADC_REFERECE, LpcdADCRefernce&0x3F);
			//2020-2-11
			ret = calibra_read_adcresult(&ADCResult);	//��ȡ������ֵ
	    printf("1--adc=%02x\r\n", ADCResult);     //92
			GainCalibraFlag = True;										//ȱʡ����У׼���

			if(ADCResult < ADCResultCenter) 	        //������ֵ��С����Ҫ��С����
		  {
				    //printf("66\r\n");
						GainCalibraFlag = False;
						while(1)
						{
						
							if(LpcdGainReduce == 0) 	        //�����Ѿ�Ϊ��Сֵ��У׼ʧ��
							{
								GainCalibraFlag = False;
								break;
							}
						
							LpcdGainReduce --; 	             //�����С������˥��						 
						
							arm_write_ext_reg(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce));	//��������
							
							
							ret = calibra_read_adcresult(&ADCResult);	//��ȡ������ֵ
										
							
							if (ADCResult > ADCResultCenter)	//У׼��ɣ������ĵ��Ƶ����ĵ��Ҳ�
							{
								GainCalibraFlag = True;
								break;
							}
						}
			}
			else
			{						
				LpcdADCRefernce = ADC_REFERNCE_MAX;	//�ο���ѹ����Ϊ���ֵ--127
				arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);//0
				arm_write_ext_reg(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);//63
				
				ret = calibra_read_adcresult(&ADCResult);//��ȡ������ֵ
				printf("2--adc=%02x\r\n", ADCResult); 		
				GainCalibraFlag = True;                  //ȱʡ����У׼���
				//printf("77\r\n");
				
				if (ADCResult > ADCResultCenter) //�ж��Ƿ����̫С�����̫Сlpcd_gain�Ŵ�
				{
					GainCalibraFlag = False;  
					while(1)
					{
					  //�����ǰ�Ѿ���������棬��Уʧ��
						if (LpcdGainAmplify == 0x7)
						{
							  GainCalibraFlag = False;//�����Ѿ�Ϊ���ֵ��У׼ʧ��
							  break;
						}
						else
						{
							  LpcdGainAmplify++; //����Ӵ�
						}
						
						arm_write_ext_reg(JREG_LPCD_CTRL4,((LpcdGainAmplify << 2) + LpcdGainReduce)); //��������
									
						ret = calibra_read_adcresult(&ADCResult);	//��ȡ������ֵ
						printf("Amplify=%02x\r\n", LpcdGainAmplify);	
  					printf("3--adc=%02x\r\n", ADCResult); 	
					  //�����У�ɹ��������ĵ��Ƶ����ĵ����
						if(ADCResult < ADCResultCenter)
						{
								GainCalibraFlag = True;	 	//У׼���
								break;
						}
					}

				}
			}
       
			//��������Уʧ�ܣ���ʧ��
			if(GainCalibraFlag == False)
			{		
				(*calibraflag) = False;
				return ADCResult;	   //У׼ʧ�ܣ����ط�����ֵ
			}
      
			//ɨ��ο���ѹֵ���ҵ����ʵĴ�Խ���ĵ������
			(*calibraflag) = False;	//������
			
			//��У����ADC�Ĳο���ѹ�趨
			ADCResult_Pre = ADCResult; //���������ֵ
			
       
      printf("111\r\n");			
      //ɨ������ݣ�ͨ����ѭ������ȷ�����յĲο���ѹ��ADC���ıȽ�ֵ			
			for(LpcdADCRefernce = ADC_REFERNCE_MIN; LpcdADCRefernce < ADC_REFERNCE_MAX; LpcdADCRefernce++)
			{
				//���òο���ѹֵ
				printf("adcrf=%02x\r\n",LpcdADCRefernce);
				arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
				
				arm_write_ext_reg(JREG_LPCD_ADC_REFERECE, LpcdADCRefernce&0x3F);
				
				ADCResult_Pre = ADCResult;	//���������ֵ�����ݷ�����Ϣ
			  
				ret = calibra_read_adcresult(&ADCResult);	//��ȡ������ֵ
				//printf("Adc[%d]=%02x\r\n",LpcdADCRefernce, ADCResult);
				//������ȿ�ʼ������ֵС����Ϊ��ʼ������ֵ��
				if(ADCResult < ADCResultCenter)
				{
//					printf("adc_pe=%02x\r\n", ADCResult_Pre);
//					printf("11Adc=%02x\r\n",ADCResult);
//					printf("adccenter=%02x\r\n", ADCResultCenter);
					//���̳ɹ�
					(*calibraflag) = True;
				  
					if((ADCResultCenter-ADCResult) < (ADCResult_Pre-ADCResultCenter)) 	//�жϵ�ǰ������ֵ��ǰһ�η�����ֵ���ĸ����ӽ����ĵ�
					{
							ADCResultCenter = ADCResult;	//��ǰ������ֵ��Ϊ�ο����ĵ㣬��Ϊ��ǰһ�θ��������õ����ĵ�
//					    printf("hh\r\n");
					}
					else
					{
						//printf("kk\r\n");
						ADCResultCenter = ADCResult_Pre; //ǰһ�η�����ֵ��Ϊ�ο����ĵ�
					
						LpcdADCRefernce--;	             //��С�ο���ѹ
					
						arm_write_ext_reg(JREG_LPCD_BIAS_CURRENT,((LpcdADCRefernce&0x40)<<5)+LpcdBiasCurrent&0x7);
						
						arm_write_ext_reg(JREG_LPCD_ADC_REFERECE,LpcdADCRefernce&0x3F);//һֱ�ڸı�			
					}
					
					//������ֵ//���������ȼ��
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
			
			clear_ext_bitmask (JREG_LPCD_MISC,BFL_JBIT_CALIB_VMID_EN);             //�ر�CalibVmidEn
			if (GainCalibraFlag == False)
			{		
				(*calibraflag) = False;
				return ADCResult;                                                   //У׼ʧ�ܣ����ط�����ֵ
			}
			return OK;
}

//��У����ȡLPCD������ֵ
unsigned char calibra_read_adcresult(unsigned char *adcresult)
{
      unsigned char ret;
			//ʹ��У׼
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN);
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN);
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_CALIBRA_EN);
			
			//�ȴ�У׼�ж�
			ret = wait_for_lpcdirq(JBIT_CALIB_IRQ);	
			
	    //�رյ�Уģʽ
			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_CALIBRA_EN);
	
	   //��ȡ������ֵ��Ϣ
			read_lpcd_adcresult(adcresult);
			return OK;
}

//�ȴ�У׼�ж�
unsigned char wait_for_lpcdirq(unsigned char irqtype)
{
      unsigned char ExtRegData;
	    unsigned char TimeOutCount;
	
			TimeOutCount = 0;
			ExtRegData = arm_read_ext_reg(JREG_LPCD_IRQ);// ��ʾirqtype=JBIT_CALIB_IRQ-->calib ��Уģʽ���һ�ο����
      
			while ((ExtRegData & irqtype) != irqtype)
			{
				ExtRegData = arm_read_ext_reg(JREG_LPCD_IRQ);				
				delay_ms(1);//��ʱ10ms
				TimeOutCount++;
				
				if  (TimeOutCount > IRQ_TIMEOUT)  
				{
					return ERROR; // ��ʱ����
				}
			}
			printf("etd=%02x\r\n", ExtRegData);
			return OK;
}

//��ȡ������ֵ
unsigned char read_lpcd_adcresult(unsigned char *adcresult)
{
        unsigned char  ExtRegData;
				unsigned char  ADCResultTemp;

				*adcresult = 0;
				ExtRegData = arm_read_ext_reg(JREG_LPCD_ADC_RESULT_H);//��ȡ��λ�ֽڣ�BIT7��BIT6��
				ADCResultTemp = ((ExtRegData & 0x3) << 6);
				ExtRegData = arm_read_ext_reg(JREG_LPCD_ADC_RESULT_L);//��ȡ��λ�ֽڣ�BIT5~BIT0��
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
	       
				lpcd_param_init();						    //LPCD������ʼ��
				lpcd_reg_init();       				    //LPCD�Ĵ�����ʼ��
			  //LpcdAuxSelect(ON);						  //����AUX�۲�ͨ��
				lpcd_init_calibra(&CalibraFlag);  //LPCD��ʼ������

				if (CalibraFlag == True)				  //������̳ɹ���������
				{
					fm17500_hard_powerdown(1);			//����LPCDģʽ
				}
				else
				{
				   printf("Calibraflag fail ***\r\n");
				}
				//delay_ms(2); 
				//return;
}

/*********************************************/
//��������	    fm17500_hard_powerdown
//���ܣ�	      Ӳ���͹��Ĳ���
//���������	  mode=1 ����͹���ģʽ, mode=0 �˳��͹���ģʽ
//����ֵ��	    OK������͹���ģʽ��
//				      ERROR���˳��͹���ģʽ��
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
		 return OK; //����͹���ģʽ
}

void lpcd_cardin_irqhandler(void)
{
//      delay_ms(20); 
//			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_CLR+JBIT_LPCD_RSTN); 	//��λLPCD�Ĵ���
//			arm_write_ext_reg(JREG_LPCD_CTRL1, JBIT_BIT_CTRL_SET+JBIT_LPCD_RSTN);   //��λ�ſ�LPCD�Ĵ���
//	     
	    
			//��Ϊÿ�λ��ѣ����ԼĴ����ᱻ��λ
		  //LpcdAuxSelect(ON);					
			//��Ҫ��ʱ�������п������һ���Ĵ���д����

//			LED0 = 1;
//			delay_ms(1000);
//			LED0 = 0;
//	    LED1   = 1;
			//printf("Card in\r\n");
	    //printf("t33=%02x\r\n",arm_read_ext_reg(0x09));
	   // printf("irq=%02x\r\n",arm_read_ext_reg(0x12));
			//--------------------------------------------------
			
			//--------------------------------------------------
			//�û��ڴ�����жϿ�Ƭ����֮��Ĵ�������request��
			//Ҳ���Ը��ݿ�Ƭ�Ƿ���ʵ���������µ����Զ���У����
			//--------------------------------------------------
	    type_A_request();
			   
			//fm17500_hard_powerdown(1);	//��������LPCDģʽ
			//delay_ms(5);
		  //return;
}
