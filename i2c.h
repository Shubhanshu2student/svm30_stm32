#ifndef __I2C_H
#define __I2C_H

#include "stm32f0xx.h"
#include <string.h>
#include <stdbool.h>
enum transmission_tag
{
  i2c_bus_free,
  i2c_bus_busy
};

void I2C1_init(void);
void I2C1_gpio_init(void);
void air_quality_init(void);
void air_quality_measure(void);
void air_quality_read(uint16_t tvoc_ppb, uint16_t co2_ppm);
void shtc_measure(uint32_t* temperature,uint32_t* humidity);
uint32_t sensirion_calc_absolute_humidity(const uint32_t *temperature,
                                          const uint32_t *humidity);
bool sgp30_set_absolute_humidity(uint32_t absolute_humidity);
void svm_compensate_rht(uint32_t* temperature, uint32_t* humidity);
void svm30_measure_event(void);
uint16_t get_temperature(void);
uint16_t get_humidity(void);
uint16_t get_tvoc(void);
uint16_t get_co2(void);
#endif /* __I2C_H */

