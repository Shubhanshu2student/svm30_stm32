#include "i2c.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_rcc.h"
#include "timer.h"
#include "i2c_handler.h"

uint8_t *g_txbuff, *g_rxbuff;
uint8_t g_rx_size;

void
I2C1_gpio_init ()
{

  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_I2C1, ENABLE);
  RCC_I2CCLKConfig (RCC_I2C1CLK_HSI);
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init (GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init (GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig (GPIOB, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig (GPIOB, GPIO_PinSource7, GPIO_AF_1);

}

uint8_t tx_data[3], rx_data[10];
void
I2C1_init ()
{
  I2C1_gpio_init ();
  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Timing = 0x2000090E;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init (I2C1, &I2C_InitStructure);
  I2C_Cmd (I2C1, ENABLE);
  I2C_AutoEndCmd (I2C1, ENABLE);

  I2C_ITConfig (I2C1, I2C_IT_TXI | I2C_IT_TCI | I2C_IT_RXNE | I2C_IT_NACKF,
                ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);
}

uint8_t j = 0, k = 0;

void
I2C1_IRQHandler ()
{
  uint8_t rdata = 0;
  if (I2C_GetITStatus (I2C1, I2C_IT_TXIS) == SET
      && I2C_GetFlagStatus (I2C1, I2C_FLAG_TXE) == SET)
  {
    I2C_SendData (I2C1, *g_txbuff++);
  }
  else if (I2C_GetITStatus (I2C1, I2C_IT_TCI) == SET)
  {
    I2C_GenerateSTOP (I2C1, ENABLE);
  }
  else if (I2C_GetITStatus (I2C1, I2C_IT_RXNE) == SET
      && I2C_GetFlagStatus (I2C1, I2C_FLAG_RXNE) == SET)
  {
    rdata = I2C_ReceiveData (I2C1);
    g_rxbuff[k++] = rdata;

    g_rx_size--;
    if (g_rx_size == 0)
    {
      k = 0;
    }
  }
  else if (I2C_GetITStatus (I2C1, I2C_IT_NACKF) == SET)
  {
    I2C_ClearFlag (I2C1, I2C_FLAG_NACKF);
    I2C_ClearFlag (I2C1, I2C_FLAG_STOPF);
  }
}

void
i2c_write (uint8_t address, uint8_t *buff, uint16_t size)
{
  I2C1->CR2 = (uint32_t) (((uint32_t) (address << 1) & I2C_CR2_SADD)
      | ((size << 16) & I2C_CR2_NBYTES) | I2C_SoftEnd_Mode
      | I2C_Generate_Start_Write);
  g_txbuff = buff;
}

void
i2c_read (uint8_t address, uint8_t *buff, uint16_t size)
{
  I2C1->CR2 = (uint32_t) (((uint32_t) (address << 1) & I2C_CR2_SADD)
      | ((size << 16) & I2C_CR2_NBYTES) | I2C_AutoEnd_Mode
      | I2C_Generate_Start_Read );
  g_rx_size = size;
  g_rxbuff = buff;
}
