#ifndef __FM17550_H
#define __FM17550_H


#define OK  1
#define ERROR 0

#define TRUE  1
#define FALSE 0

#define IN_HPD  1
#define OUT_HPD 0

#define IN_DPD  1
#define OUT_DPD 0

#define FIFOADDR 0x09
#define W_FIFOADDR  (FIFOADDR << 1) & 0x7E
#define R_FIFOADDR  ((FIFOADDR << 1) & 0X7E) | 0x80

/*
//fm17550�Ĵ���ֵ���壬����17550 datasheet
#define CommandReg_W	0x02//0x01  //������ֹͣ����ִ��
#define CommandReg_R	0x82
//#define CommandReg    0X01
#define SoftReset     0x0f  //��λFM17550 //�ο�fm17550ָ������
#define ControlReg_W  0x18//0x0c  //���ƼĴ���
#define ControlReg_R  0x98

#define TxModeReg_W   0x24//0x12
#define TxModeReg_R   0xA4

#define TxControlReg_W  0x28//0x14
#define TxControlReg_R  0x88

#define RxModeReg_W   0x26//0x13
#define RxModeReg_R   0xA6
#define Status2Reg_W	0x10//0x08
#define Status2Reg_R  0x90
#define BitFramingReg_W  0x1A//0x0D
#define BitFramingReg_R  0x9A
//#define TModeReg    0x2A
//#define TPrescalerReg 0x2B
#define TPrescalerReg_W 0x56//0x2B
#define TPrescalerReg_R 0xD6

#define TReloadMSBReg_W 0x58//0x2C
#define TReloadMSBReg_R 0xD8

#define TReloadLSBReg_W 0x5A //0x2D
#define TReloadLSBReg_R 0xDA

#define Transceive	0x0C //����FIFO���������ݵ����ߣ����ڷ�����Զ����������
#define FIFOLevelReg_W 0x14//0x0A //fifo���ƼĴ���
#define FIFOLevelReg_R 0x94
#define FIFODataReg_W  0x12//0x09 //fifo���ݼĴ���
#define FIFODataReg_R  0x92
#define Idle         0x00 //����״̬
#define WaterLevelReg_W 0x16//0x0B//fifo�����������Ĵ���
#define ComIrqReg_W   0x08//0x04
#define ComIrqReg_R   0x88//0x04//�����ж�����λ--��
#define TModeReg_W    0x54//0x2A//��ʱ�����üĴ���
#define TModeReg_R    0xD4
#define TxAutoReg_W   0x2A
#define TxAutoReg_R   0xAA
#define GsNReg_W    0x4E
#define GsNReg_R    0xCE
#define GsNOffReg_W 0x46
#define GsNOffReg_R 0xC6
#define RFCfgReg_W  0x4C
*/
//fm17550�Ĵ���ֵ���壬����17550 datasheet
#define Command_reg 					0x01
#define comiend_reg           0x02//�ж�����ʹ�����ֹ����λ
#define divien_reg            0x03//�ж�����ʹ�����ֹ����λ
#define Divirq_reg            0x05
#define softreset       			0x0f
#define control_reg       	  0x0c//���ƼĴ���
#define txmode_reg       			0x12
#define txcontrol_reg   			0x14
#define rxmode_reg       			0x13
#define Status1_reg           0x07
#define status2_reg     			0x08
#define bitframing_reg   			0x0D
#define tprescaler_reg       	0x2B
#define treloadmsb_reg        0x2C
#define treloadlsb_reg      	0x2D
#define Transceive       			0x0C//����
#define Transmit              0x04//����FIFO�������е�����
#define fifolevel_reg       	0x0A//FIFO���ݳ���
#define fifodata_reg       		0x09
#define fifodata_reg_w        0x12

#define Idle			       			0x00
#define waterlevel_reg       	0x0B
#define comirq_reg       			0x04
#define tmode_reg       			0x2A//��ʱ�����üĴ���
#define txauto_reg       			0x15
#define GsN_reg       			  0x27
#define cwgsp_reg             0x28
#define modgsp_reg            0x29
#define GsNoff_reg       			0x23
#define RFCfg_reg       			0x26
#define Mode_reg              0x11
#define CRCResultMSB_reg      0x21
#define CRCResultLSB_reg      0x22
#define Error_reg             0x06
#define coll_reg              0x0e
#define VersionReg  0x37//��ѯоƬ�汾

//====================================��չ�Ĵ�������==============================================
unsigned char arm_write_ext_reg(unsigned char reg_add, unsigned char reg_value);//д����չ�Ĵ���
unsigned char arm_read_ext_reg(unsigned char reg_add);                          //��ȡ��չ�Ĵ���
unsigned char set_ext_bitmask(unsigned char reg_add, unsigned char mask);       //������չ�Ĵ�����־λ
unsigned char clear_ext_bitmask(unsigned char reg_add, unsigned  char mask);    //�����չ�Ĵ�����־λ
//================================================================================================
unsigned char pcd_set_timer(unsigned long delaytime);
unsigned char fm17550_soft_reset(void);
unsigned char fm17550_hard_reset(void);
unsigned char set_bit_mask(unsigned char reg_add, unsigned char mask);
unsigned  char clear_bit_mask(unsigned char reg_add, unsigned char mask);
unsigned char clear_fifo(void);
unsigned char pcd_command(unsigned char Command, unsigned char *pInData, unsigned char InLenByte, unsigned char *pOutData, unsigned int *pOutLenBit);
unsigned char get_uid(unsigned char Command, unsigned char *pInData, unsigned char InLenByte, unsigned char *pOutData, unsigned int *pOutLenBit);
void pcd_tx_on(void);//��������
void pcd_tx_off(void);//�ر�����
unsigned char set_rf(unsigned char mode);
unsigned char pcd_configiso_type(unsigned char  type);//ѡ��ģʽA/B
void type_A_request(void);
void set_card_to_idle(void);
unsigned char fm17550_softpowerdown(void);//����͹���ģʽ
unsigned char fm17550_hardpowerdown(unsigned char mode);//Ӳ���͹���ģʽ
unsigned char fm17550_deeppowerdown(unsigned char mode);//���˯��ģʽ

void test_init(void);//����lpcd���ܳ�ʼ������
unsigned char crc_cal(unsigned char len,  unsigned char *rec_buff);
#endif

