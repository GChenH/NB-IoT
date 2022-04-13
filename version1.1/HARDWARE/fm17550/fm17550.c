#include "process_task.h"
#include "fm17550.h"

extern unsigned char card_[5];


//CRC����
unsigned char crc_cal(unsigned char len,  unsigned char *rec_buff)
{
    	spi1_hard_write_reg(Mode_reg, 0x01);     //MSB,6363
			//spi1_write_fifo(len, snd_buff);        //haltд��FIFO //50 00
			spi1_hard_write_reg(Command_reg, 0x03);  //cmd=0x03  ����CRC����
			//delay_ms(1000);
	   
//	    rec_buff[0] = spi1_hard_read_reg(CRCResultLSB_reg);  //crc��λ�����ȷ���
//	    rec_buff[1] = spi1_hard_read_reg(CRCResultMSB_reg);  //crc��λ������ʱ���������
      spi1_hard_write_reg(Command_reg, Idle);
      return 0;	
}


/*********************************************/
//��������	    arm_write_ext_reg
//���ܣ�	      д����չ�Ĵ���
//���������	  reg_add:�Ĵ�����ַ��reg_value:�Ĵ�����ֵ
//����ֵ��	    OK / ERROR
/*********************************************/
unsigned char arm_write_ext_reg(unsigned char reg_add, unsigned char reg_value)
{
    spi1_hard_write_reg(0x0F, 0x40 + reg_add);
    spi1_hard_write_reg(0x0F, 0xC0 + reg_value);
    return OK;
}

/*********************************************/
//��������	    arm_read_ext_reg
//���ܣ�	      ��ȡ��չ�Ĵ���
//���������	  reg_add���Ĵ�����ַ
//����ֵ��	    �Ĵ�����ֵ
/*********************************************/
unsigned char arm_read_ext_reg(unsigned char reg_add)
{
    spi1_hard_write_reg(0x0F, 0x80 + reg_add);
    return spi1_hard_read_reg(0x0F);
}

//������չ�Ĵ�����־λ
unsigned char set_ext_bitmask(unsigned char reg_add, unsigned char mask)
{
    unsigned char result;

    result = arm_write_ext_reg(reg_add, arm_read_ext_reg(reg_add) | mask);  // set bit mask
    return result;
}

//�����չ�Ĵ�����־λ
unsigned char clear_ext_bitmask(unsigned char reg_add, unsigned char mask)
{
    unsigned char  result;
    result = arm_write_ext_reg(reg_add, arm_read_ext_reg(reg_add) & (~mask)); // clear bit mask
    return result;
}

//CARD TO IDLE
void set_card_to_idle(void)
{
   unsigned char reqa_buf[1] = {0};
	 reqa_buf[0] = 0x26;
	 spi1_hard_write_reg(bitframing_reg, 0x07);
	 spi1_hard_write_reg(Command_reg, Transmit);
	 spi1_write_fifo(1, reqa_buf);
   set_bit_mask(bitframing_reg, 0x80);//��������
}

//send REQA/WUPA
void type_A_request(void)
{
    unsigned char send_information[10]; //��������
    unsigned char rx_information[10]; //���յ�������
    unsigned char result = 0;
    unsigned int rec_bitlen = 0;
    unsigned char uid_buf[2] = {0};
		
    pcd_configiso_type(0);            //ѡ��TypeA Э��ģʽ
    clear_bit_mask(txmode_reg, 0x80); //�ر�tx crc
    clear_bit_mask(rxmode_reg, 0x80); //�ر�rx crx
    set_bit_mask(rxmode_reg, 0x08);   //bit3-RxNoErr��λ,FIFO�н��յ���Ч����ʱcommirqreg��rxirq����һ    
    clear_bit_mask(status2_reg, 0x08);//���ƽ�������������������ģʽ�������״̬λ
    set_rf(3);                        //��������ѡ��TX1,TX2���
    delay_ms(50);
    send_information[0] = 0x52;
    //pcd_set_timer(1);//���ö�ʱ����ʱ����,�����ÿɲ����ã�����������ö�ʱ����ʱҲ����������ȡ����
    result = pcd_command(Transceive, send_information, 1, rx_information, &rec_bitlen);//Transceive==0x0c

    if(result == OK)
    {
			 clear_fifo();
       uid_buf[0] = 0x93;
			 uid_buf[1] = 0x20;
			 clear_bit_mask(coll_reg, 0x80);
			 get_uid(Transceive, uid_buf, 2, rx_information, &rec_bitlen);
    }

}

//ѡ��type A/B
unsigned char pcd_configiso_type(unsigned char  type)
{
    if (type == 0)//ISO14443_A
    {

        spi1_hard_write_reg(control_reg, spi1_hard_read_reg(control_reg) | 0x10);//���ó�readerģʽ
        spi1_hard_write_reg(txauto_reg, spi1_hard_read_reg(txauto_reg) | 0x40); //���ó�100%ASK��Ч
        
			  spi1_hard_write_reg(txmode_reg, 0x00);   //TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE A
        spi1_hard_write_reg(rxmode_reg, 0x00);   //RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE A
        //CHECK = 0;
			  //spi1_hard_write_reg(txmode_reg, 0x80); //���ݷ���ʱʹ��CRCУ��
			  //CHECK = 1;
			  //spi1_hard_write_reg(rxmode_reg, 0x00); //���ݽ���ʱʹ��CRCУ��
			
			  spi1_hard_write_reg(RFCfg_reg, 0x7F);    // ��������48dB
        //printf("rfc=%02x\r\n", spi1_hard_read_reg(RFCfg_reg));
        
			  spi1_hard_write_reg(GsN_reg, 0xff);
			  //spi1_hard_write_reg(GsN_reg, 0x88);       //GsNonReg(0x27)//���ù���
        spi1_hard_write_reg(cwgsp_reg, 0x3f);   //CWGsPReg(0x28)
			  //spi1_hard_write_reg(cwgsp_reg, 0x10);
			
			  spi1_hard_write_reg(modgsp_reg, 0x3f);
        //(modgsp_reg, 0x10);  //ModGsPReg(0x29)
        
    }

    if (type == 1)//ISO14443_B
    {
        spi1_hard_write_reg(control_reg, 0x10); //ControlReg 0x0C ����readerģʽ
        spi1_hard_write_reg(txmode_reg, 0x83); //TxModeReg 0x12 ����TX CRC��Ч��TX FRAMING =TYPE B
        spi1_hard_write_reg(rxmode_reg, 0x83); //RxModeReg 0x13 ����RX CRC��Ч��RX FRAMING =TYPE B
        spi1_hard_write_reg(GsN_reg, 0xF4); //GsNReg 0x27 ����ON�絼
        spi1_hard_write_reg(GsNoff_reg, 0xF4); //GsNOffReg 0x23 ����OFF�絼
        spi1_hard_write_reg(txauto_reg, 0x00);// TxASKReg 0x15 ����100%ASK��Ч
    }

    return OK;
}
//������Ƶ���
/*
 mode,��Ƶ���ģʽ
 0��  �ر����
 1��  ����TX1���
 2��  ����TX2���
 3��  TX1,TX2�������Tx2Ϊ�������
 ����ֵ��OK
*/
unsigned char set_rf(unsigned char mode)
{
    unsigned char result = 0;

    if((spi1_hard_read_reg(txcontrol_reg) & 0x03) == mode)
    {
        return OK;
    }

    if(mode == 0)
    {
        ////clear_bit_mask(TxControlReg_W, 0x03); //�ر�TX1,TX2���
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) & ~0x03);
    }

    if(mode == 1)
    {
        //clear_bit_mask(TxControlReg_W, 0x01);//����TX1���
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) & ~0x01);
    }

    if(mode == 2)
    {
        //clear_bit_mask(TxControlReg_W, 0x02);//����TX2���
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) & ~0x02);
    }

    if(mode == 3)
    {
        //set_bit_mask(TxControlReg_W, 0x03);//��TX1, TX2���
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) | 0x03);
    }

    //delay_ms(100);//�������������Ҫ��ʱ�ȴ������ز��ź��ȶ�
    return  result;
}

//��������
void pcd_tx_on(void)
{
    unsigned char temp = 0;
    temp = spi1_hard_read_reg(txcontrol_reg);

    if(!(temp & 0x03))
    {
        //set_bit_mask(TxControlReg_W, 0x03);
        spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) | 0x03);
    }
}

//�ر�����
void pcd_tx_off(void)
{
    unsigned char temp = 0;
    temp = spi1_hard_read_reg(txcontrol_reg);
    spi1_hard_write_reg(txcontrol_reg, temp & (~0x03));
}

//���FIFO
unsigned char clear_fifo(void)
{
    set_bit_mask(fifolevel_reg, 0x80); //���FIFO����

    if(spi1_hard_read_reg(fifolevel_reg) == 0)
    {
        return OK;
    }
    else
    {
        return ERROR;
    }
}
//�����־λ
unsigned  char clear_bit_mask(unsigned char reg_add, unsigned char mask)
{
    unsigned char result;
    result = spi1_hard_write_reg(reg_add, spi1_hard_read_reg(reg_add) & ~mask);
    return  result;
}
//���ñ�־λ
unsigned char set_bit_mask(unsigned char reg_addr, unsigned char mask)
{
    unsigned char result = 0;
    result = spi1_hard_write_reg(reg_addr, spi1_hard_read_reg(reg_addr) | mask);
    return result;
}
//�����λ17550
unsigned char fm17550_soft_reset(void)
{
    spi1_hard_write_reg(Command_reg, softreset);      //�����λ17550
    return  set_bit_mask(control_reg, 0x10);    //����17550Ϊ��д��ģʽ
}

//��λ17550
unsigned char fm17550_hard_reset(void)
{
    NPD_RST = 0;
    delay_us(100);
    NPD_RST = 1;
    delay_us(100);

    return OK;
}

//���ö�ʱ����ʱ
unsigned char pcd_set_timer(unsigned long delaytime)//�趨��ʱʱ�䣨ms��
{
    unsigned long time_reload = 0;
    unsigned int prescaler = 0;

    while(prescaler < 0xffff)
    {
        time_reload = ((delaytime * (long)13560) - 1) / (prescaler * 2 + 1);

        if(time_reload < 0xffff)
        {
            break;
        }

        prescaler++;
    }

    time_reload = time_reload & 0xFFFF;
    //set_bit_mask(TModeReg_W, prescaler >> 8);
    spi1_hard_write_reg(tmode_reg, spi1_hard_read_reg(tmode_reg) | (prescaler >> 8));
    spi1_hard_write_reg(tprescaler_reg, prescaler & 0xFF);
    spi1_hard_write_reg(treloadmsb_reg, time_reload >> 8);
    spi1_hard_write_reg(treloadlsb_reg, time_reload & 0xFF);

    return OK;
}
unsigned char get_uid(unsigned char command, unsigned char *send_inf_data, unsigned char send_len_byte, unsigned char *rec_data, unsigned int *pOutLenBit)
{
    unsigned char  rec_fifo_len = 0;    //��ʱ�����ֽڳ���
    unsigned char i = 0;
	  unsigned int t = 0;
    unsigned char tx_buf[3];

	  clear_fifo();
	  spi1_hard_write_reg(bitframing_reg, 0x00);
    spi1_hard_write_reg(Command_reg, Idle);
    spi1_hard_write_reg(waterlevel_reg, 0x20);  
	  spi1_hard_write_reg(comirq_reg, 0x7F);     //�Ĵ���commirqreg�еı�־λ�����
    //delay_ms(5);
    spi1_hard_write_reg(Command_reg, command);
    if(command == Transceive)
    {
        spi1_write_fifo(send_len_byte, send_inf_data);		
        set_bit_mask(bitframing_reg, 0x80);//��������
			  spi1_hard_write_reg(comirq_reg, 0x04); 
       		  
			  while((spi1_hard_read_reg(comirq_reg) & 0x20) != 0x20)
				{
				   t++;
					 if(t == 0x2000)
					 {
					    t = 0;
						  printf("888\r\n");
						  return FALSE;
					 }
				}
					
				rec_fifo_len = spi1_hard_read_reg(fifolevel_reg);
				spi1_read_fifo(rec_fifo_len, rec_data);
				for(i = 0; i < rec_fifo_len; i++)
				{
						card_[i] = rec_data[i];
						printf("%02x  ", rec_data[i]);
				}
				rec_fifo_len = 0;
				//set_card_to_idle();
	  }
	  
    printf("\n");
    return OK;  
}
//PCDͨ�ź���
unsigned char pcd_command(unsigned char command, unsigned char *send_inf_data, unsigned char send_len_byte, unsigned char *rec_data, unsigned int *pOutLenBit)
{
    //unsigned char  result;
    unsigned char  rec_fifo_len = 0;    //��ʱ�����ֽڳ���
    //unsigned char  irq  = 0;
    unsigned char i = 0;
	  unsigned int j = 0;
    unsigned char tx_buf[3];
    
	  unsigned char len = 0;
	  unsigned char buf[2] = {0};
  	//clear_fifo();
    spi1_hard_write_reg(Command_reg, Idle);
	  spi1_hard_write_reg(bitframing_reg, 0x07);         //���һ�ֽڷ��Ͷ��ٱ���
    spi1_hard_write_reg(waterlevel_reg, 0x20);  	
	  spi1_hard_write_reg(comirq_reg, 0x7F);             //�Ĵ���commirqreg�еı�־λ�����
    spi1_hard_write_reg(Command_reg, command);

    if(command == Transceive)
    { 
        spi1_write_fifo(send_len_byte, send_inf_data);	
        set_bit_mask(bitframing_reg, 0x80);            //��������
			
			  while((spi1_hard_read_reg(comirq_reg) & 0x20) != 0x20)
				{
				   j++;
					 if(j == 0x3000)
					 {
					    j = 0;
						  printf("666\r\n");
						  return FALSE;
					 }
				}
			   		
				rec_fifo_len = spi1_hard_read_reg(fifolevel_reg);
//			  printf("len=%02x\r\n", rec_fifo_len);
//				spi1_read_fifo(rec_fifo_len, rec_data);
//				for(i = 0; i < rec_fifo_len; i++)
//				{
//						printf("%02x  ", rec_data[i]);
//				}
				rec_fifo_len = 0;
	  } 
		
    return OK;
}

//����͹��Ĳ���������ģʽ��ֻ�رվ�����������ԭ��״̬��
unsigned char fm17550_softpowerdown(void)
{
    unsigned char tmp;                           //״̬��־
    tmp = spi1_hard_read_reg(Command_reg) & 0x10;//command_reg powerdownλ��1����sotfpowerdown�͹���ģʽ

    if(tmp)
    {
        clear_bit_mask(Command_reg, 0x10);       //�˳�sotf power down ģʽ
        return ERROR;
    }
    else
    {
        set_bit_mask(Command_reg, 0x10);         //����soft power down ģʽ
    }

    return OK;
}

//Ӳ���͹��Ĳ���������ģʽ��
unsigned char fm17550_hardpowerdown(unsigned char mode)
{
    if(mode == 1)
    {
        NPD_RST = 0;//NPD = 0
        //arm_write_ext_reg(0x01, 0x01);//LpcdEn = 0
        return IN_HPD;
    }
    else
    {
        NPD_RST = 1;
        //delay_ms(200);
        //printf("****");
        //GPIO_SetBits(GPIOE, GPIO_Pin_5);
        //fm17550_hard_reset();//��λ
        return OUT_HPD;
    }
}

//��ȵ͹���ģʽ��deep power down��
//1���룬0�˳�
unsigned char fm17550_deeppowerdown(unsigned char mode)
{
    if(mode)
    {
        NPD_RST = 0;//NPD=0
        //arm_write_ext_reg(0x01, 0x01);//LpcdEn = 0
        return IN_DPD;
    }
    else
    {
        NPD_RST = 1;

        return OUT_DPD;
    }
}
