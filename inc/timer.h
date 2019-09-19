/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#ifndef __timer_H
#define __timer_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f0xx.h"
#include "timercfg.h"

typedef void (*timer_callback_function) (void);

void 
timer_init();

void 
timer_set(enum timer_id_tag timer_id, uint32_t period, 
          timer_callback_function fnptr);

void 
timer_process(void);

void 
SysTick_Handler(void);
#endif
