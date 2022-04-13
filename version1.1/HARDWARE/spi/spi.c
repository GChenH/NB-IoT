//PA4--CS
//PA5--CLK
//PA6--MISO
//PA7--MOSI
#include "process_task.h"

//spi1初始化
void spi1_init(void)
{
   SPI_InitTypeDef spi_struct;
	 GPIO_InitTypeDef gpio_struct;
	
	 //使能GPIO,SPI时钟
	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	 //引脚复用
	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	 //初始化GPIO
	 gpio_struct.GPIO_Mode = GPIO_Mode_AF;
	 gpio_struct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7; //PA5/6/7
	 gpio_struct.GPIO_OType = GPIO_OType_PP;
	 gpio_struct.GPIO_PuPd = GPIO_PuPd_UP;
	 //gpio_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 gpio_struct.GPIO_Speed = GPIO_Speed_40MHz;
	 GPIO_Init(GPIOA, &gpio_struct);
	
	 gpio_struct.GPIO_Mode = GPIO_Mode_OUT;                       //PA4
	 gpio_struct.GPIO_Pin = GPIO_Pin_4;
	 gpio_struct.GPIO_OType = GPIO_OType_PP;
	 gpio_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 gpio_struct.GPIO_Speed = GPIO_Speed_40MHz;
	 GPIO_Init(GPIOA, &gpio_struct);
	 
	 //NPD_RST = 1;
	 SPI1_CS = 1;
 
	 gpio_struct.GPIO_Mode = GPIO_Mode_OUT;                       //PA0--fm17550复位引脚
	 gpio_struct.GPIO_Pin = GPIO_Pin_0;
	 gpio_struct.GPIO_OType = GPIO_OType_PP;
	 gpio_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 gpio_struct.GPIO_Speed = GPIO_Speed_40MHz;
	 GPIO_Init(GPIOA, &gpio_struct);
	 
	 spi_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	 spi_struct.SPI_Mode = SPI_Mode_Master;
	 spi_struct.SPI_DataSize = SPI_DataSize_8b;
	 spi_struct.SPI_CPOL = SPI_CPOL_Low;
	 spi_struct.SPI_CPHA = SPI_CPHA_1Edge;
	 spi_struct.SPI_NSS = SPI_NSS_Soft;
	 spi_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	 spi_struct.SPI_FirstBit = SPI_FirstBit_MSB;
	 spi_struct.SPI_CRCPolynomial = 7;
	 SPI_Init(SPI1, &spi_struct);
	 SPI_Cmd(SPI1, ENABLE);
   
   //SPI1_CS = 1;	 
}

//设置spi频率函数
void SPI1_SetSpeed(unsigned char SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    SPI1->CR1 &= 0XFFC7;
    SPI1->CR1 |= SPI_BaudRatePrescaler;	                           //设置SPI1速度
	  
    SPI_Cmd(SPI1, ENABLE);
}

unsigned char spi1_hard_write_reg(unsigned char addr, unsigned char value)
{
    addr = ( addr << 1 ) & 0x7E;
    SPI1_CS = 0;
    SPI_I2S_SendData(SPI1, addr);                                  //send spi2 addr data
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);//清掉第一字节接收内容，为无效数据
    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    SPI_I2S_ReceiveData(SPI1);                                     //not care data
    SPI_I2S_SendData(SPI1, value);
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    SPI_I2S_ReceiveData(SPI1);
	  SPI1_CS = 1;

    return  OK;
}

unsigned char  spi1_hard_read_reg(unsigned char addr)
{
    uint8_t receive = 0;
    addr = (addr << 1)  | 0x80;
    SPI1_CS = 0;
    SPI_I2S_SendData(SPI1, addr);

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    SPI_I2S_ReceiveData(SPI1);
    //SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);

    SPI_I2S_SendData(SPI1, 0x00);

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    receive = SPI_I2S_ReceiveData(SPI1);
    SPI1_CS = 1;

    return receive;
}

//连续写FIFO
void spi1_write_fifo(unsigned char send_len, unsigned char *val)
{
    unsigned char i = 0;
    unsigned char tmpdata = 0;

    SPI1_CS = 0;
    SPI_I2S_SendData(SPI1, W_FIFOADDR);                            //send spi2 addr data
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);//清掉第一字节接收内容，为无效数据

    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    SPI_I2S_ReceiveData(SPI1);                                     //not care data

    for(i = 0; i < send_len; i++)
    {
        tmpdata = *(val + i);
        SPI_I2S_SendData(SPI1, tmpdata);
        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        tmpdata = SPI_I2S_ReceiveData(SPI1);
    }
    SPI1_CS = 1;

    return ;
}
//连续读FIFO
void spi1_read_fifo(unsigned char rec_len, unsigned char *val)
{
    unsigned char i = 0;
    unsigned char tmpdata = 0;

    SPI1_CS = 0;
    SPI_I2S_SendData(SPI1, R_FIFOADDR);

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    tmpdata = SPI_I2S_ReceiveData(SPI1);            //not care data
	  //tmpdata = tmpdata;
    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);

    for(i = 0; i < (rec_len - 1); i++)
    {
        SPI_I2S_SendData(SPI1, R_FIFOADDR);

        while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

        SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
        *(val + i) = SPI_I2S_ReceiveData(SPI1);
    }

    SPI_I2S_SendData(SPI1, 0x00);

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
    *(val + i) = SPI_I2S_ReceiveData(SPI1);
    SPI1_CS = 1;

    return;
}

