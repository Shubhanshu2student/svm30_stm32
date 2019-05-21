#include "main.h"
#include "i2c.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_rcc.h"
#include "sensirion_common.h"
#include "timer.h"

#define SGP30_ADDRESS 0x58
#define SHTC1_ADDRESS 0x70

enum transmission_tag transmission_state;
uint8_t g_length , g_i2c_rx_buffer[10];
uint8_t *g_i2c_tx_buffer;
unsigned char buffer[10];
uint64_t g_serial_id_sgp30;
uint16_t g_tvoc_ppb, g_co2_ppm;
uint8_t data[10],crc_1st ;
uint16_t data_word[5];
uint32_t g_humidity, g_temperature;
uint32_t abs_humidity,values[4];
uint16_t g_h2_raw, g_eth_raw;
int32_t raw_humidity, raw_temperature;

#define T_LO (-20000)
#define T_HI 70000

static const uint32_t AH_LUT_100RH[] = {1078,  2364,  4849,  9383,   17243,
30264, 50983, 82785, 130048, 198277};
static const uint32_t T_STEP = (T_HI - T_LO) /((sizeof(AH_LUT_100RH)/sizeof(*(AH_LUT_100RH))) - 1);


void I2C1_gpio_init()
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//GPIO_Mode_OUT
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //GPIO_OType_PP
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO0_PuPd_UP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C1_init()
{
  I2C1_gpio_init();
  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Timing = 0x2000090E;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1,&I2C_InitStructure);
  I2C_Cmd(I2C1, ENABLE);
  //  I2C_AutoEndCmd(I2C1, ENABLE);
  //  I2C_ITConfig(I2C1, I2C_IT_RXI, ENABLE);
  //  
  //  NVIC_InitTypeDef NVIC_InitStructure;
  //  NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
  //  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  //  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //  NVIC_Init(&NVIC_InitStructure);
  //  NVIC_EnableIRQ(I2C1_IRQn);
  //  NVIC_SetPriority(I2C1_IRQn,0);
  transmission_state = i2c_bus_free;
}

//void I2C1_IRQHandler()
//{
//  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == SET)
//  {
//    if(g_length > 0)
//    {
//      I2C_SendData(I2C1, *g_i2c_tx_buffer);
//      g_length--;
//      g_i2c_tx_buffer++;
//      while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);
//    }
//  }
//  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == SET)
//  {
//    for(uint8_t i = 0; i < g_length; i++)
//    {
//      g_i2c_rx_buffer[i] = I2C_ReceiveData(I2C1);
//    }
//  }
//  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
//  I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
//  transmission_state = i2c_bus_free;
//}

void air_quality_init()
{
  buffer[0] = 0x20; /*Init Air Sensor*/
  buffer[1] = 0x03;
  crc_1st = sensirion_common_generate_crc(buffer,2);
  buffer[2]= crc_1st;
  I2C_TransferHandling(I2C1, (SGP30_ADDRESS & 0x7f)<< 1, 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
  for(uint8_t i = 0; i < 3; i++)
    I2C_SendData(I2C1, buffer[i]);
 
  ms_Delay(10);
 
  buffer[0] = 0x20;     /*Start measurement*/
  buffer[1] = 0x08;
  crc_1st = sensirion_common_generate_crc(buffer,2);
  buffer[2]= crc_1st;
  I2C_TransferHandling(I2C1, (SGP30_ADDRESS & 0x7f) << 1, 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
  for(uint8_t i = 0; i < 3 ; i++)
    I2C_SendData(I2C1, buffer[i]);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
}



void air_quality_read(uint16_t tvoc_ppb, uint16_t co2_ppm)
{
  uint8_t data_ptr=0,j = 0,i, data_byte[10];
  
  I2C_TransferHandling(I2C1, SGP30_ADDRESS << 1, 6, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  if(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == SET)
  {
    while(data_ptr !=6)
    {
      data[data_ptr] = I2C_ReceiveData(I2C1);
      data_ptr++;
      while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
    }
  }
  
  for (i = 0; i < 7; i += 3 ) 
  {
    uint8_t ret= sensirion_common_check_crc(&data[i], 2, data[i+2]);
    if (ret == STATUS_OK)
    {
      data_byte[j++] = data[i];
      data_byte[j++] = data[i+1];
    }
  }
  data_word[0] = data_byte[0]<<8 +data_byte[1];
  data_word[1] = data_byte[2]<<8+data_byte[3];
  tvoc_ppb = data_word[1];
  co2_ppm = data_word[0];
  
  I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}

void shtc_measure(uint32_t temperature, uint32_t humidity)
{
  /******************************************************/
  uint8_t j = 0,data_ptr=0;
  
  buffer[0] = 0x58;  /*RH command for SHTC1*/
  buffer[1] = 0xE0;
  crc_1st = sensirion_common_generate_crc(buffer, 2);
  buffer[2] = crc_1st;
  I2C_TransferHandling(I2C1, (SHTC1_ADDRESS & 0x7f) << 1, 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
  for(uint8_t i = 0;i<3;i++)
  {
    I2C_SendData(I2C1, buffer[i]);
  }
  ms_Delay(15);
  
  memset(&buffer, 0, 10);
  memset(&data, 0, 10);
  I2C_TransferHandling(I2C1, (SHTC1_ADDRESS & 0x7f) << 1, 3, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);
  while(data_ptr !=3)
  {
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);
    buffer[data_ptr] = I2C_ReceiveData(I2C1);
    data_ptr++;
  }
  j = 0;
  for (uint8_t i = 0; i < 3; i += 3 )
  {
    uint8_t ret= sensirion_common_check_crc(&buffer[i], 2, buffer[i+2]);
    if (ret == STATUS_OK)
    {
      data[j++] = buffer[i];
      data[j++] = buffer[i + 1];
    }
  }
  raw_humidity = data[0]<<8 | data[1];
  //  humidity = 100*(g_raw_humidity/65535.0);
  //  humidity = (103.7 - 3.2*(g_raw_temp/65535))*(g_raw_humidity/65535.0);
  
  /************************************************/
  buffer[0] = 0x78;  /*T command for SHTC1,0x7866 */
  buffer[1] = 0x66;
  crc_1st = sensirion_common_generate_crc(buffer, 2);
  buffer[2] = crc_1st;
  I2C_TransferHandling(I2C1, (SHTC1_ADDRESS & 0x7f) << 1, 3, I2C_SoftEnd_Mode, 
                       I2C_Generate_Start_Write);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
  for(uint8_t i = 0; i < 3; i++)
  {
    I2C_SendData(I2C1, buffer[i]);
  }
  ms_Delay(15);
  j=0,data_ptr = 0;
  memset(&buffer, 0, 10);
  memset(&data, 0, 10);
  I2C_TransferHandling(I2C1, (SHTC1_ADDRESS & 0x7f) << 1, 3, I2C_AutoEnd_Mode, 
                       I2C_Generate_Start_Read);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);
  while(data_ptr !=3)
  {
    while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);
    buffer[data_ptr] = I2C_ReceiveData(I2C1);
    data_ptr++;
  }
  
  for (uint8_t i = 0; i < 3; i += 3 )
  {
    uint8_t ret= sensirion_common_check_crc(&buffer[i], 2, buffer[i+2]);
    if (ret == STATUS_OK)
    {
      data[j++] = buffer[i];
      data[j++] = buffer[i + 1];
    }
  }
  raw_temperature = data[0]<<8 | data[1];
  //  temperature = -45.68 + (175.7*(g_raw_temp/65535.0));
  g_temperature = (((21875 * raw_temperature) >> 13) - 45000)/100;
  g_humidity = ((12500* raw_humidity )>>13)/100;
  temperature = g_temperature;
  humidity = g_humidity;
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
}

uint32_t sensirion_calc_absolute_humidity(const int32_t temperature,
                                          const int32_t humidity)
{
  uint32_t t, norm_humi, i, rem,  ret;
  
  if (humidity == 0)
    return 0;
  
  norm_humi = ((uint32_t)humidity * 82) >> 13;
  t = temperature - T_LO;
  i = t / T_STEP;
  rem = t % T_STEP;
  
  if (i >= ARRAY_SIZE(AH_LUT_100RH) - 1) {
    ret = AH_LUT_100RH[ARRAY_SIZE(AH_LUT_100RH) - 1];
    
  } else if (rem == 0) {
    ret = AH_LUT_100RH[i];
    
  } else {
    ret = (AH_LUT_100RH[i] +
           ((AH_LUT_100RH[i + 1] - AH_LUT_100RH[i]) * rem / T_STEP));
  }
  return ret * norm_humi / 1000;
}

bool sgp30_set_absolute_humidity(uint32_t absolute_humidity)
{
  uint64_t ah = absolute_humidity;
  uint16_t ah_scaled;
  if (absolute_humidity > 256000)
    return false;
  
  /* ah_scaled = (ah / 1000) * 256 */
  ah_scaled = (uint16_t)((ah * 256 * 16777) >> 24);
//  sensirion_i2c_write_cmd_with_args(SGP_I2C_ADDRESS, 0x2061, &ah_scaled, 2);
  buffer[0] = ah_scaled & 0x00FF;  /*T command for SHTC1,0x7866 */
  buffer[1] = ah_scaled & 0xFF00;
  crc_1st = sensirion_common_generate_crc(buffer, 2);
  buffer[2] = crc_1st;
  I2C_TransferHandling(I2C1, (SGP30_ADDRESS & 0x7f) << 1, 3, I2C_SoftEnd_Mode, 
                       I2C_Generate_Start_Write);
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
  for(uint8_t i = 0; i < 3; i++)
  {
    I2C_SendData(I2C1, buffer[i]);
  }
  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
  return true;
}

void svm_compensate_rht(int32_t temperature, int32_t humidity)
{
  temperature = ((temperature * 8225) >> 13) - 500;
  humidity = (humidity * 8397) >> 13;
}