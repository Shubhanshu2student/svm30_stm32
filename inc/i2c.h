#ifndef __I2C_H
#define __I2C_H

#include "stm32f0xx.h"
#include <string.h>
#include <stdbool.h>
#include "i2c_handler.h"

void
I2C1_init (void);
void
I2C1_gpio_init (void);
void
I2C1_IRQHandler (void);
void
i2c_write (uint8_t address, uint8_t *buff, uint16_t size);
void
i2c_read (uint8_t address, uint8_t *buff, uint16_t size);

#endif /* __I2C_H */

