#ifndef __PROCESS_TASK_H
#define __PROCESS_TASK_H

#include "led.h"
#include "delay.h"
#include "usart.h"
#include "exti.h"
#include "sys.h"
#include "spi.h"
#include "fm17550.h"
#include "lpcd.h"

//进入stop模式宏
#define ENTER_STOP_MODE         0
//一体低功耗环境宏
#define NORMAL_LOWPER_MODE      1
//普通测试宏
#define NORMAL_MODE             0
#define SPI1_CS  PAout(4)
#define NPD_RST  PAout(0)

void process_task(void);
void bsp_init(void);
void into_stop_mode_init(void);
void fm17550_init(void);
void lpcd_func(void);
void gpio_a2(void);
#endif

