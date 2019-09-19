#include "svm30.h"
#include "stm32f0xx.h"
#include "i2c.h"
#include "sensirion_common.h"
#include "timer.h"
#include "sensor-card-data.h"

#define SGP30_ADDRESS 0x58
#define SHTC1_ADDRESS 0x70

uint8_t sgp30_init[] = { 0x20, 0x03, 0x14 };
uint8_t sgp30_get_baseline[] = { 0x20, 0x15, 0xEB };
uint8_t sgp30_set_baseline[8];
uint8_t sgp30_set_humidity[] = { 0x20, 0x61, 0x00, 0x00, 0x00 };
uint8_t sgp30_measure_test[] = { 0x20, 0x32, 0xFA };
uint8_t sgp30_get_ft_set_version[] = { 0x20, 0x2f, 0xf5 };
uint8_t sgp30_measure_raw_signal[] = { 0x20, 0x50, 0x23 };
uint8_t shtc1_hum_read_cmd[] = { 0x58, 0xE0, 0x90 };
uint8_t shtc1_temp_read[] = { 0x78, 0x66, 0x90 };
uint8_t sgp30_measure[] = { 0x20, 0x08, 0xE4 };
uint8_t sgp30_soft_reset[] = {0x00,0x06};
struct sensor_card_command_tag sensor_card_com_st;

uint8_t sgp30_read_baseline[6];
uint8_t sgp30_co2_tvoc[6];
uint8_t shtc1_raw_humidity[3];
uint8_t shtc1_raw_temperature[3];
uint8_t set_ft_ver[3];
uint32_t g_humidity, g_temperature = 0;
uint16_t g_co2_ppm, g_tvoc_ppb,g_read_co2_base_value,g_read_tvoc_base_value;
uint8_t g_baseline_cmd = 0;
int32_t raw_humidity, raw_temperature = 0;
uint32_t abs_humidity = 0;
uint16_t g_read_baseline_value,co2_value_baseline,tvoc_value_baseline = 0;

#define T_LO (-20000)
#define T_HI 70000

static const uint32_t AH_LUT_100RH[] =
{ 1078, 2364, 4849, 9383, 17243, 30264, 50983, 82785, 130048, 198277 };
static const uint32_t T_STEP = (T_HI - T_LO)
    / ((sizeof(AH_LUT_100RH) / sizeof(*(AH_LUT_100RH))) - 1);

void
air_quality_init ()
{
  timer_set (air_sensor_timer, 20, air_quality_process);
}

void
air_quality_process ()
{
  static uint8_t i = 0;
  static uint8_t sgp30_uninit_status = 1;
  uint32_t u32_temp, u32_hum;
  static uint8_t one_sec_timer = 0;
  static uint8_t one_sec_flag, soft_reset_flag = 0;
  static uint8_t set_baseLine_flag = 1;
  static uint16_t u1615sectimer = 0;
  one_sec_timer++;
  
  if (one_sec_timer == 50)
  {
    one_sec_timer = 0;
    one_sec_flag = 1;
  }
  if (!one_sec_flag)
  {
    return;
  }

  if (i >= 12 )
  {
    i = 0;
    one_sec_flag = 0;
    return;
  }
  if (sgp30_uninit_status)
  {
    i2c_write (SGP30_ADDRESS, sgp30_init, 3);       //write_init
    sgp30_uninit_status = 0;
    return;
  }
  if(u1615sectimer < 1000)
  {
    u1615sectimer++;
  }
  if(u1615sectimer >= 1000)
  {
      if (i == 0)
      {
        i2c_write (SHTC1_ADDRESS, shtc1_hum_read_cmd, 3);  //hum_read_comm
        i++;
        return;
      }
      if (i == 1)
      {
        i2c_read (SHTC1_ADDRESS, shtc1_raw_humidity, 3);  //hum_value
        i++;
        return;
      }
      if (i == 2)
      {
        i2c_write (SHTC1_ADDRESS, shtc1_temp_read, 3);  //temp_read_comm
        uint8_t u8j = 0, u8data_temp[2] = { 0 };
        for (uint8_t u8a = 0; u8a < 3; u8a += 3)
        {
          uint8_t ret = sensirion_common_check_crc (&shtc1_raw_humidity[u8a], 2,
                                                    shtc1_raw_humidity[u8a + 2]);
          if (ret == STATUS_OK)
          {
            u8data_temp[u8j++] = shtc1_raw_humidity[u8a];
            u8data_temp[u8j] = shtc1_raw_humidity[u8a + 1];
          }
        }
        raw_humidity = u8data_temp[0] << 8 | u8data_temp[1];
        i++;
        return;
      }
      if (i == 3)
      {
        i2c_read (SHTC1_ADDRESS, shtc1_raw_temperature, 3); //temp_value
        i++;
        return;
      }
      if (i == 4)
      {
        uint8_t u8j = 0, u8_data[2] =
        { 0 };
        for (uint8_t u8a = 0; u8a <= 3; u8a += 3)
        {
          uint8_t ret = sensirion_common_check_crc (&shtc1_raw_temperature[u8a], 2,
                                                    shtc1_raw_temperature[u8a + 2]);
          if (ret == STATUS_OK)
          {
            u8_data[u8j++] = shtc1_raw_temperature[u8a];
            u8_data[u8j++] = shtc1_raw_temperature[u8a + 1];
          }
        }
        raw_temperature = u8_data[0] << 8 | u8_data[1];

        g_temperature = (((21875 * raw_temperature) >> 13) - 45000) / 100;
        g_humidity = ((12500 * raw_humidity) >> 13) / 100;

        u32_temp = ((g_temperature * 8225) >> 13) - 500;
        u32_hum = (g_humidity * 8397) >> 13;

        abs_humidity = sensirion_calc_absolute_humidity (&u32_temp, &u32_hum);

        uint64_t ah = abs_humidity;
        uint16_t ah_scaled;
        if (abs_humidity < 256000)
        {
          ah_scaled = (uint16_t) ((ah * 256 * 16777) >> 24);

          sgp30_set_humidity[2] = (uint8_t) ((ah_scaled & 0xFF00) >> 8);
          sgp30_set_humidity[3] = (uint8_t) ((ah_scaled & 0x00FF) >> 0);
          sgp30_set_humidity[4] = sensirion_common_generate_crc (
              &sgp30_set_humidity[2], 2);

          i2c_write (SGP30_ADDRESS, sgp30_set_humidity, 5);   //set_humidity
        }
        i++;
        return;
      }
      if (i == 5)
      {
        i2c_write (SGP30_ADDRESS, sgp30_measure, 3);  //co2_read_com
        i++;
        return;
      }
      if (i == 6)
      {
        i2c_read (SGP30_ADDRESS, sgp30_co2_tvoc, 6);  //co2_value
        i++;
        return;
      }
      if (i == 7)
      {
        uint8_t u8j = 0, data_byte[4];
        uint16_t data_word[2];
        for (uint8_t u8a = 0; u8a < 6; u8a += 3)
        {
          uint8_t ret = sensirion_common_check_crc (&sgp30_co2_tvoc[u8a], 2,
                                                    sgp30_co2_tvoc[u8a + 2]);
          if (ret == 0)
          {
            data_byte[u8j++] = sgp30_co2_tvoc[u8a];
            data_byte[u8j++] = sgp30_co2_tvoc[u8a + 1];
          }
        }
        data_word[0] = data_byte[0] << 8 | data_byte[1];
        data_word[1] = data_byte[2] << 8 | data_byte[3];
        g_tvoc_ppb = data_word[1];
        g_co2_ppm = data_word[0];
        i++;
        return;
      }

      if (i == 8 )
      {
        i2c_write (SGP30_ADDRESS, sgp30_get_baseline, 3);
        i++;
        return;
      }
      if (i == 9)
      {
        i2c_read (SGP30_ADDRESS, sgp30_read_baseline, 6);
        i++;
        return;
      }
      if (i == 10 && g_baseline_cmd != get_baseline)
      {
        uint8_t u8j = 0, data_byte[4] = { 0 };
        uint16_t data_word[2] = { 0 };
        for (uint8_t u8a = 0; u8a < 6; u8a += 3)
        {
          uint8_t ret = sensirion_common_check_crc (&sgp30_read_baseline[u8a], 2,
                                                    sgp30_read_baseline[u8a + 2]);
          if (ret == 0)
          {
            data_byte[u8j++] = sgp30_read_baseline[u8a];
            data_byte[u8j++] = sgp30_read_baseline[u8a + 1];
          }
        }
        data_word[0] = data_byte[0] << 8 | data_byte[1];
        data_word[1] = data_byte[2] << 8 | data_byte[3];
        g_read_co2_base_value = data_word[0];
        g_read_tvoc_base_value = data_word[1];
        g_baseline_cmd = 0;
        i++;
        return;
      }
      if(i == 11)
      {
        if(set_baseLine_flag && g_read_co2_base_value != 0)
        {
        co2_value_baseline = g_read_co2_base_value ;
        tvoc_value_baseline = g_read_tvoc_base_value ;
        sgp30_set_baseline[0] = 0x20;
        sgp30_set_baseline[1] = 0x16;
        sgp30_set_baseline[2] = tvoc_value_baseline >> 8;
        sgp30_set_baseline[3] = tvoc_value_baseline;
        sgp30_set_baseline[4] = sensirion_common_generate_crc(&sgp30_set_baseline[2], 2);
        sgp30_set_baseline[5] = co2_value_baseline >> 8;
        sgp30_set_baseline[6] = co2_value_baseline;
        sgp30_set_baseline[7] = sensirion_common_generate_crc(&sgp30_set_baseline[5], 2);

//        i2c_write (SGP30_ADDRESS, sgp30_set_baseline, 8);
          set_baseLine_flag = 0;
        }
        i++;
        return;
      }
  }
}

uint32_t
sensirion_calc_absolute_humidity (const uint32_t *temperature,
                                  const uint32_t *humidity)
{
  uint32_t t = 0, norm_humi = 0, i = 0, rem = 0, ret;

  if (*humidity == 0)
    return 0;

  norm_humi = ((uint32_t) *humidity * 82) >> 13;
  t = *temperature - T_LO;
  i = t / T_STEP;
  rem = t % T_STEP;

  if (i >= ARRAY_SIZE(AH_LUT_100RH) - 1)
  {
    ret = AH_LUT_100RH[ARRAY_SIZE(AH_LUT_100RH) - 1];

  }
  else if (rem == 0)
  {
    ret = AH_LUT_100RH[i];

  }
  else
  {
    ret = (AH_LUT_100RH[i]
        + ((AH_LUT_100RH[i + 1] - AH_LUT_100RH[i]) * rem / T_STEP));
  }
  return ret * norm_humi / 1000;
}

uint16_t
get_temperature ()
{
  return g_temperature;
}

uint16_t
get_humidity ()
{
  return g_humidity;
}

uint16_t
get_tvoc ()
{
  return g_tvoc_ppb;
}

uint16_t
get_co2 ()
{
  return g_co2_ppm;
}

uint16_t
get_read_co2_baseline ()
{
  return g_read_co2_base_value;
}

uint16_t
get_read_tvoc_baseline ()
{
  return g_read_tvoc_base_value;
}
