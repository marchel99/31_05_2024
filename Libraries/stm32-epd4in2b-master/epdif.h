#ifndef EPDIF_H
#define EPDIF_H

#include "stm32l4xx_hal.h"

void DigitalWrite(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state);
GPIO_PinState DigitalRead(GPIO_TypeDef* port, uint16_t pin);
void SpiTransfer(uint8_t data);
void DelayMs(uint32_t ms);
void EpdIf_IfInit(void);

#endif /* EPDIF_H */
