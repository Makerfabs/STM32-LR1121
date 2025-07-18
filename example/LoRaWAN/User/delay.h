#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
#endif /* DELAY_H */