#ifndef DHT11_H
#define DHT11_H

#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"


#define DHT11_IO        GPIOC
#define DHT11_PIN       GPIO_PIN_10


void DHT11_Start(void);
void DHT_Read(void);

#endif

