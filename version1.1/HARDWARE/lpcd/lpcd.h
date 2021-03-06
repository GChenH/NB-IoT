#ifndef __LPCD_H
#define __LPCD_H

#include "fm17550.h"
#include "spi.h"



#define   True                     1
#define   False                    0

#define  	TIMER1_CFG  	  			   3			//0  t1 = 500ms
#define  	TIMER2_CFG  	  			   13			//2  t2 = 1.5ms
#define  	TIMER3_CFG  	  			   11		  //1  t3 = 47us
#define  	LPCD_TX2RFEN				     1			//2
#define		LPCD_CWN					       0x1		//3
#define		LPCD_CWP					       0x3		//4
#define		LPCD_THRESHOLD_RATIO	   4			//5
#define		LPCD_AUTO_DETECT_TIMES	 1			//6
#define  	LPCD_IE 					       0x01		//7
#define		AUTO_WUP_EN					     1			//
#define		AUTO_WUP_CFG				     2			//10,15分钟自动唤醒一次,100--1小时自动唤醒一次

#define  	LPCD_AUX_EN	    			   1   		  //0
#define  	LPCD_BIAS_CURRENT   		 0x00   	//1


#define	 	IRQ_TIMEOUT					      9			//3

#define	 	LP10K_CALIBRA_THRESHOLD 	17		//4
#define		ADC_CENTER_RANGE			    5			//5
#define	 	LPCD_SAMPLE_CLK  			    1			//6
#define		LPCD_MODE					        0x0		//7
#define		LP_OSC_MODE					      0			//8
#define		WUP_OSC_MODE				      0			//9
#define		VMID_BG_CFG					      8			//10
//#define		VMID_BG_CFG					      5	
//#define		VMID_BG_CFG					      11	
#define		ERR_TRIG_JUDGE_RANGE		  5			//11





//============================================================================
#define		JREG_EXT_REG_ENTRANCE		0x0F	//ext register entrance
//============================================================================
#define		JBIT_EXT_REG_WR_ADDR		0X40	//wrire address cycle
#define		JBIT_EXT_REG_RD_ADDR		0X80	//read address cycle
#define		JBIT_EXT_REG_WR_DATA		0XC0	//write data cycle
#define		JBIT_EXT_REG_RD_DATA		0X00	//read data cycle


//============================================================================
#define		JREG_LVD_CTRL				  0x1D	//Low Votage Detect register
//============================================================================

//============================================================================
#define		JREG_LPCD_CTRL1				0x01	//Lpcd Ctrl register1
//============================================================================
#define		JBIT_LPCD_EN				  0x01	//enble LPCD
#define		JBIT_LPCD_RSTN				0X02	//lpcd reset
#define		JBIT_LPCD_CALIBRA_EN	0x04	//into lpcd calibra mode
#define		JBIT_LPCD_CMP_1				0x08	//Compare times 1 or 3
#define		JBIT_LPCD_IE				  0x10	//Enable LPCD IE
#define		JBIT_BIT_CTRL_SET			0x20	//lpcdctrl1 控制寄存器bitC位置1
#define		JBIT_BIT_CTRL_CLR			0x00	//Lpcdctrl1 控制寄存器bitC位置0
//============================================================================

//============================================================================
#define		JREG_LPCD_CTRL2				0x02	//Lpcd Ctrl register2
//============================================================================

//============================================================================
#define		JREG_LPCD_CTRL3				0x03	//Lpcd Ctrl register3
//============================================================================

//============================================================================

//============================================================================
#define		JREG_LPCD_CTRL4				0x04	//Lpcd Ctrl register4
//============================================================================

//============================================================================

//============================================================================
#define		JREG_LPCD_BIAS_CURRENT		0x05	//Lpcd bias current register
//============================================================================

//============================================================================
#define		JREG_LPCD_ADC_REFERECE		0x06	//Lpcd adc reference register 
//============================================================================

//============================================================================
#define		JREG_LPCD_T1CFG				0x07	//T1Cfg[3:0] register 
//============================================================================

//============================================================================
#define		JREG_LPCD_T2CFG				0x08	//T2Cfg[4:0] register 
//============================================================================

//============================================================================
#define		JREG_LPCD_T3CFG				0x09	//T2Cfg[4:0] register 
//============================================================================

//============================================================================
#define		JREG_LPCD_VMIDBD_CFG		0x0A	//VmidBdCfg[4:0] register 
//============================================================================

//============================================================================
#define		JREG_LPCD_AUTO_WUP_CFG			0x0B	//Auto_Wup_Cfg register 
//============================================================================

//============================================================================
#define		JREG_LPCD_ADC_RESULT_L			0x0C	//ADCResult[5:0] Register 
//============================================================================

//============================================================================
#define		JREG_LPCD_ADC_RESULT_H			0x0D	//ADCResult[7:6] Register 
//============================================================================

//============================================================================
#define		JREG_LPCD_THRESHOLD_MIN_L		0x0E	//LpcdThreshold_L[5:0] Register 
//============================================================================

//============================================================================
#define		JREG_LPCD_THRESHOLD_MIN_H		0x0F	//LpcdThreshold_L[7:6] Register 
//============================================================================

//============================================================================
#define		JREG_LPCD_THRESHOLD_MAX_L		0x10	//LpcdThreshold_H[5:0] Register 
//============================================================================

//============================================================================
#define		JREG_LPCD_THRESHOLD_MAX_H		0x11	//LpcdThreshold_H[7:6] Register 
//============================================================================

//============================================================================
#define		JREG_LPCD_IRQ				0x12	//LpcdStatus Register 
//============================================================================
#define		JBIT_CARD_IN_IRQ			0x01	//irq of card in
#define		JBIT_LPCD23_IRQ				0x02	//irq of LPCD 23 end
#define		JBIT_CALIB_IRQ				0x04	//irq of calib end
#define		JBIT_LP10K_TESTOK_IRQ		0x08	//irq of lp osc 10K ok
#define		JBIT_AUTO_WUP_IRQ			0x10	//irq of Auto wake up 
//============================================================================

//============================================================================
#define		JREG_LPCD_RFT1				0x13	//Aux1 select Register 
//============================================================================
//#define		BFL_JBIT_AUX1_CLOSE				0x00	//close aux1 out
//#define		BFL_JBIT_AUX1_VDEM_LPCD			0x01	//vdem of lpcd
//#define		BFL_JBIT_AUX1_VP_LPCD			0x02	//voltage of charecap
//============================================================================

//============================================================================
#define		JREG_LPCD_RFT2				0x14	//Aux2 select Register 
//============================================================================
//#define		BFL_JBIT_AUX2_CLOSE				0x00	//close aux1 out
//#define		BFL_JBIT_AUX2_VDEM_LPCD			0x01	//vdem of lpcd
//#define		BFL_JBIT_AUX2_VP_LPCD			0x02	//voltage of charecap
//============================================================================

//============================================================================
#define		JREG_LPCD_RFT3				0x15	//LPCD test1 Register 
//============================================================================
//#define		BFL_JBIT_LP_OSC10K_EN			0x01	//enable lp osc10k
//#define		BFL_JBIT_LP_OSC_CALIBRA_EN		0x02	//enable lp osc10k calibra mode
//#define		BFL_JBIT_LP_CURR_TEST			0x04	//enable lp t1 current test
//#define		BFL_JBIT_LPCD_TEST2_LPCD_OUT	0x08	//lpcd_test2[3]:LPCD_OUT 
//============================================================================

//============================================================================
#define		JREG_LPCD_RFT4				0x16	//LPCD test2 Register 
//============================================================================
//#define		BFL_JBIT_T1_OUT_EN				0x01	//D5:T1_OUT
//#define		BFL_JBIT_OSCCLK_OUT_EN			0x02	//D4:OSC_CLK_OUT
//#define		BFL_JBIT_OSCEN_OUT_EN			0x04	//D3:OSC_EN
//#define		BFL_JBIT_LP_CLK_LPCD_OUT_EN		0x08	//D2:LP_CLK or LPCD_OUT
//#define		BFL_JBIT_T3_OUT_EN				0x10	//D1:T3_OUT
//============================================================================		

//============================================================================
//#define		BFL_JREG_LP_CLK_CNT1			0x17	//lp_clk_cnt[5:0] Register 
//============================================================================

//============================================================================
//#define		BFL_JREG_LP_CLK_CNT2			0x18	//lp_clk_cnt[7:6] Register 
//============================================================================

//============================================================================
//#define		BFL_JREG_VERSIONREG2			0x19	//VersionReg2[1:0] Register 
//============================================================================

//============================================================================
//#define		BFL_JREG_IRQ_BAK				0x1A	//Irq bak Register 
//============================================================================
//#define		BFL_JBIT_IRQ_INV_BAK			0x01	//Irq Inv backup
//#define		BFL_JBIT_IRQ_PUSHPULL_BAK		0x02	//Irq pushpull backup
//============================================================================


//============================================================================
#define		JREG_LPCD_RFT5				0x1B	//LPCD TEST3 Register 
//============================================================================
//#define		BFL_JBIT_LPCD_TESTEN			0x01	//lPCD test mode
//#define		BFL_JBIT_AWUP_TSEL				0x02	//Auto wakeup test mode
//#define		BFL_JBIT_RNG_MODE_SEL			0x04	//RNG  mode sel
//#define		BFL_JBIT_USE_RET				0x08	//use retention mode
//============================================================================

//============================================================================
#define		JREG_LPCD_MISC				      0x1C	//LPCD Misc Register 
//============================================================================
#define		BFL_JBIT_CALIB_VMID_EN			0x01	//lPCD test mode
#define		BFL_JBIT_AMP_EN_SEL				  0x04	//LPCD amp en select

//============================================================================

#define ADC_REFERNCE_MIN 0x0	//实际并联电容值最小
#define ADC_REFERNCE_MAX 0x7F	//实际电容值最大
//============================================================================


unsigned char lpcd_param_init(void); //lpcd参数初始化
unsigned char lpcd_reg_init(void);   //lpcd寄存器初始化
unsigned char lpcd_init_calibra(unsigned char *calibraflag);//lpcd初始化调校
unsigned char calibra_read_adcresult(unsigned char *adcresult);
unsigned char wait_for_lpcdirq(unsigned char irqtype);
unsigned char read_lpcd_adcresult(unsigned char *adcresult);
void lpcd_auto_wakeup_irq_handler(void); //自动唤醒中断
void lpcd_cardin_irqhandler(void); //卡片进场中断
unsigned char fm17500_hard_powerdown(unsigned char mode);

#endif

