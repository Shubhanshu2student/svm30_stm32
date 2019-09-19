/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/


#include "timer.h"


enum timer_status_tag
{
  timer_not_set,
  timer_running,
  timer_expired
};

volatile static struct timer_handle_tag
{
  uint32_t u32_timer_period;
  uint32_t u32_timer_count;
  enum timer_status_tag timer_status;
  timer_callback_function callback_function;
}timer_handle[total_timers];

void 
timer_init()
{
  RCC_ClocksTypeDef RCC_Clocks;
  
  RCC_GetClocksFreq(&RCC_Clocks);
  /*internal clock for 1ms*/
  SysTick_CLKSourceConfig(RCC_Clocks.HCLK_Frequency/1000);
  
  /*Configure the SysTick IRQ priority */
  //NVIC_SetPriority(SysTick_IRQn,1);
  
  SysTick_Config(RCC_Clocks.HCLK_Frequency /1000);
  for(uint8_t idx = 0;idx<total_timers;idx++)
  {
    timer_handle[idx].timer_status = timer_not_set;
  }
}

void 
timer_set(enum timer_id_tag timer_id, uint32_t period, 
          timer_callback_function fnptr)
{
  if(timer_id >= total_timers)
  {
    return;
  }
  timer_handle[timer_id].u32_timer_count = 0;
  timer_handle[timer_id].callback_function = fnptr;
  timer_handle[timer_id].u32_timer_period = period;
  timer_handle[timer_id].timer_status = timer_running;
}

void 
timer_process(void)
{
  uint8_t u8var;
  
  for(u8var=0; u8var < total_timers; u8var++)
  {
    if(timer_handle[u8var].timer_status == timer_expired)
    {
      timer_handle[u8var].timer_status = timer_running;
      timer_handle[u8var].callback_function();
    }
  }
}

void 
SysTick_Handler(void)
{
  uint32_t timecounter = 0;
  for(uint8_t idx = 0; idx < total_timers; idx++)
  {
    if((timer_handle[idx].timer_status != timer_not_set))
    {
      timecounter = ++timer_handle[idx].u32_timer_count;
      if(timecounter == timer_handle[idx].u32_timer_period)
      {
        timer_handle[idx].u32_timer_count = 0;
        timer_handle[idx].timer_status = timer_expired;
      }
    }
  }     
}
