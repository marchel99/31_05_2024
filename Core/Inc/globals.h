#ifndef GLOBALS_H
#define GLOBALS_H

#include "stm32l4xx_hal.h"




extern volatile uint32_t encoderValue;
extern volatile int updateDisplayFlag;


int __io_putchar(int ch);

#endif // GLOBALS_H