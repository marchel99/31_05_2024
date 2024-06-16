#ifndef GLOBALS_H
#define GLOBALS_H

#include "stm32l4xx_hal.h"



int __io_putchar(int ch);
extern volatile uint32_t encoderValue;
extern volatile int updateDisplayFlag;



#endif // GLOBALS_H
