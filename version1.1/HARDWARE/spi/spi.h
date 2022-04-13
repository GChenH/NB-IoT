#ifndef __SPI_H
#define __SPI_H

void spi1_init(void);
void SPI1_SetSpeed(unsigned char SPI_BaudRatePrescaler);  //设置spi速率函数
void spi1_read_fifo(unsigned char rec_len, unsigned char *val);
void spi1_write_fifo(unsigned char send_len, unsigned char *val);
unsigned char  spi1_hard_read_reg(unsigned char addr);
unsigned char spi1_hard_write_reg(unsigned char addr, unsigned char value);

#endif

