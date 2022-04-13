#include "process_task.h"
#include "fm17550.h"

extern unsigned char card_[5];


//CRC计算
unsigned char crc_cal(unsigned char len,  unsigned char *rec_buff)
{
    	spi1_hard_write_reg(Mode_reg, 0x01);     //MSB,6363
			//spi1_write_fifo(len, snd_buff);        //halt写进FIFO //50 00
			spi1_hard_write_reg(Command_reg, 0x03);  //cmd=0x03  激活CRC计算
			//delay_ms(1000);
	   
//	    rec_buff[0] = spi1_hard_read_reg(CRCResultLSB_reg);  //crc地位，最先发送
//	    rec_buff[1] = spi1_hard_read_reg(CRCResultMSB_reg);  //crc高位，发送时放在最后面
      spi1_hard_write_reg(Command_reg, Idle);
      return 0;	
}


/*********************************************/
//函数名：	    arm_write_ext_reg
//功能：	      写入扩展寄存器
//输入参数：	  reg_add:寄存器地址；reg_value:寄存器数值
//返回值：	    OK / ERROR
/*********************************************/
unsigned char arm_write_ext_reg(unsigned char reg_add, unsigned char reg_value)
{
    spi1_hard_write_reg(0x0F, 0x40 + reg_add);
    spi1_hard_write_reg(0x0F, 0xC0 + reg_value);
    return OK;
}

/*********************************************/
//函数名：	    arm_read_ext_reg
//功能：	      读取扩展寄存器
//输入参数：	  reg_add，寄存器地址
//返回值：	    寄存器数值
/*********************************************/
unsigned char arm_read_ext_reg(unsigned char reg_add)
{
    spi1_hard_write_reg(0x0F, 0x80 + reg_add);
    return spi1_hard_read_reg(0x0F);
}

//设置扩展寄存器标志位
unsigned char set_ext_bitmask(unsigned char reg_add, unsigned char mask)
{
    unsigned char result;

    result = arm_write_ext_reg(reg_add, arm_read_ext_reg(reg_add) | mask);  // set bit mask
    return result;
}

//清除扩展寄存器标志位
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
   set_bit_mask(bitframing_reg, 0x80);//启动发送
}

//send REQA/WUPA
void type_A_request(void)
{
    unsigned char send_information[10]; //发送数据
    unsigned char rx_information[10]; //接收到的数据
    unsigned char result = 0;
    unsigned int rec_bitlen = 0;
    unsigned char uid_buf[2] = {0};
		
    pcd_configiso_type(0);            //选择TypeA 协议模式
    clear_bit_mask(txmode_reg, 0x80); //关闭tx crc
    clear_bit_mask(rxmode_reg, 0x80); //关闭rx crx
    set_bit_mask(rxmode_reg, 0x08);   //bit3-RxNoErr置位,FIFO中接收到有效数据时commirqreg的rxirq被置一    
    clear_bit_mask(status2_reg, 0x08);//控制接收器，发射器和数据模式检测器的状态位
    set_rf(3);                        //天线配置选择TX1,TX2输出
    delay_ms(50);
    send_information[0] = 0x52;
    //pcd_set_timer(1);//配置定时器超时设置,可配置可不配置，试验过不配置定时器超时也可以正常读取数据
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

//选择type A/B
unsigned char pcd_configiso_type(unsigned char  type)
{
    if (type == 0)//ISO14443_A
    {

        spi1_hard_write_reg(control_reg, spi1_hard_read_reg(control_reg) | 0x10);//设置成reader模式
        spi1_hard_write_reg(txauto_reg, spi1_hard_read_reg(txauto_reg) | 0x40); //设置成100%ASK有效
        
			  spi1_hard_write_reg(txmode_reg, 0x00);   //TxModeReg 0x12 设置TX CRC无效，TX FRAMING =TYPE A
        spi1_hard_write_reg(rxmode_reg, 0x00);   //RxModeReg 0x13 设置RX CRC无效，RX FRAMING =TYPE A
        //CHECK = 0;
			  //spi1_hard_write_reg(txmode_reg, 0x80); //数据发射时使能CRC校验
			  //CHECK = 1;
			  //spi1_hard_write_reg(rxmode_reg, 0x00); //数据接收时使能CRC校验
			
			  spi1_hard_write_reg(RFCfg_reg, 0x7F);    // 配置增益48dB
        //printf("rfc=%02x\r\n", spi1_hard_read_reg(RFCfg_reg));
        
			  spi1_hard_write_reg(GsN_reg, 0xff);
			  //spi1_hard_write_reg(GsN_reg, 0x88);       //GsNonReg(0x27)//配置功耗
        spi1_hard_write_reg(cwgsp_reg, 0x3f);   //CWGsPReg(0x28)
			  //spi1_hard_write_reg(cwgsp_reg, 0x10);
			
			  spi1_hard_write_reg(modgsp_reg, 0x3f);
        //(modgsp_reg, 0x10);  //ModGsPReg(0x29)
        
    }

    if (type == 1)//ISO14443_B
    {
        spi1_hard_write_reg(control_reg, 0x10); //ControlReg 0x0C 设置reader模式
        spi1_hard_write_reg(txmode_reg, 0x83); //TxModeReg 0x12 设置TX CRC有效，TX FRAMING =TYPE B
        spi1_hard_write_reg(rxmode_reg, 0x83); //RxModeReg 0x13 设置RX CRC有效，RX FRAMING =TYPE B
        spi1_hard_write_reg(GsN_reg, 0xF4); //GsNReg 0x27 设置ON电导
        spi1_hard_write_reg(GsNoff_reg, 0xF4); //GsNOffReg 0x23 设置OFF电导
        spi1_hard_write_reg(txauto_reg, 0x00);// TxASKReg 0x15 设置100%ASK无效
    }

    return OK;
}
//设置射频输出
/*
 mode,射频输出模式
 0，  关闭输出
 1，  仅打开TX1输出
 2，  仅打开TX2输出
 3，  TX1,TX2打开输出，Tx2为反向输出
 返回值：OK
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
        ////clear_bit_mask(TxControlReg_W, 0x03); //关闭TX1,TX2输出
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) & ~0x03);
    }

    if(mode == 1)
    {
        //clear_bit_mask(TxControlReg_W, 0x01);//仅打开TX1输出
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) & ~0x01);
    }

    if(mode == 2)
    {
        //clear_bit_mask(TxControlReg_W, 0x02);//仅打开TX2输出
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) & ~0x02);
    }

    if(mode == 3)
    {
        //set_bit_mask(TxControlReg_W, 0x03);//打开TX1, TX2输出
        result = spi1_hard_write_reg(txcontrol_reg, spi1_hard_read_reg(txcontrol_reg) | 0x03);
    }

    //delay_ms(100);//打开天线输出后需要延时等待天线载波信号稳定
    return  result;
}

//开启天线
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

//关闭天线
void pcd_tx_off(void)
{
    unsigned char temp = 0;
    temp = spi1_hard_read_reg(txcontrol_reg);
    spi1_hard_write_reg(txcontrol_reg, temp & (~0x03));
}

//清除FIFO
unsigned char clear_fifo(void)
{
    set_bit_mask(fifolevel_reg, 0x80); //清楚FIFO缓冲

    if(spi1_hard_read_reg(fifolevel_reg) == 0)
    {
        return OK;
    }
    else
    {
        return ERROR;
    }
}
//清除标志位
unsigned  char clear_bit_mask(unsigned char reg_add, unsigned char mask)
{
    unsigned char result;
    result = spi1_hard_write_reg(reg_add, spi1_hard_read_reg(reg_add) & ~mask);
    return  result;
}
//设置标志位
unsigned char set_bit_mask(unsigned char reg_addr, unsigned char mask)
{
    unsigned char result = 0;
    result = spi1_hard_write_reg(reg_addr, spi1_hard_read_reg(reg_addr) | mask);
    return result;
}
//软件复位17550
unsigned char fm17550_soft_reset(void)
{
    spi1_hard_write_reg(Command_reg, softreset);      //软件复位17550
    return  set_bit_mask(control_reg, 0x10);    //设置17550为读写器模式
}

//复位17550
unsigned char fm17550_hard_reset(void)
{
    NPD_RST = 0;
    delay_us(100);
    NPD_RST = 1;
    delay_us(100);

    return OK;
}

//设置定时器超时
unsigned char pcd_set_timer(unsigned long delaytime)//设定超时时间（ms）
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
    unsigned char  rec_fifo_len = 0;    //临时数据字节长度
    unsigned char i = 0;
	  unsigned int t = 0;
    unsigned char tx_buf[3];

	  clear_fifo();
	  spi1_hard_write_reg(bitframing_reg, 0x00);
    spi1_hard_write_reg(Command_reg, Idle);
    spi1_hard_write_reg(waterlevel_reg, 0x20);  
	  spi1_hard_write_reg(comirq_reg, 0x7F);     //寄存器commirqreg中的标志位被清除
    //delay_ms(5);
    spi1_hard_write_reg(Command_reg, command);
    if(command == Transceive)
    {
        spi1_write_fifo(send_len_byte, send_inf_data);		
        set_bit_mask(bitframing_reg, 0x80);//启动发送
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
//PCD通信函数
unsigned char pcd_command(unsigned char command, unsigned char *send_inf_data, unsigned char send_len_byte, unsigned char *rec_data, unsigned int *pOutLenBit)
{
    //unsigned char  result;
    unsigned char  rec_fifo_len = 0;    //临时数据字节长度
    //unsigned char  irq  = 0;
    unsigned char i = 0;
	  unsigned int j = 0;
    unsigned char tx_buf[3];
    
	  unsigned char len = 0;
	  unsigned char buf[2] = {0};
  	//clear_fifo();
    spi1_hard_write_reg(Command_reg, Idle);
	  spi1_hard_write_reg(bitframing_reg, 0x07);         //最后一字节发送多少比特
    spi1_hard_write_reg(waterlevel_reg, 0x20);  	
	  spi1_hard_write_reg(comirq_reg, 0x7F);             //寄存器commirqreg中的标志位被清除
    spi1_hard_write_reg(Command_reg, command);

    if(command == Transceive)
    { 
        spi1_write_fifo(send_len_byte, send_inf_data);	
        set_bit_mask(bitframing_reg, 0x80);            //启动发送
			
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

//软件低功耗操作（待机模式，只关闭晶振，其他保持原样状态）
unsigned char fm17550_softpowerdown(void)
{
    unsigned char tmp;                           //状态标志
    tmp = spi1_hard_read_reg(Command_reg) & 0x10;//command_reg powerdown位置1进入sotfpowerdown低功耗模式

    if(tmp)
    {
        clear_bit_mask(Command_reg, 0x10);       //退出sotf power down 模式
        return ERROR;
    }
    else
    {
        set_bit_mask(Command_reg, 0x10);         //进入soft power down 模式
    }

    return OK;
}

//硬件低功耗操作（掉电模式）
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
        //fm17550_hard_reset();//复位
        return OUT_HPD;
    }
}

//深度低功耗模式（deep power down）
//1进入，0退出
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
