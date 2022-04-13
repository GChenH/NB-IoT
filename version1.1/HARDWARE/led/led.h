#ifndef __LED_H
#define __LED_H

#include "stm32l1xx.h"
#include "sys.h"

#define LED0  PAout(11)
void led_init(void);

#endif
