/******************************************************************************
 Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
 ******************************************************************************/
#ifndef __SVM30_H
#define __SVM30_H

#include <stdint.h>
#include <stdbool.h>

void
air_quality_init (void);

uint32_t
sensirion_calc_absolute_humidity (const uint32_t *temperature,
                                  const uint32_t *humidity);
void
air_quality_process (void);

uint16_t
get_co2 ();

uint16_t
get_humidity ();

uint16_t
get_tvoc ();

uint16_t
get_temperature ();

uint16_t
get_read_co2_baseline ();

uint16_t
get_read_tvoc_baseline ();
#endif /*__SVM30_H*/
