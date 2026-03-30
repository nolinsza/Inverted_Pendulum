#include <stdio.h>
#include <stdint.h>
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_adc/adc_oneshot.h>
#include "esp_log.h"

#define TAG "ADC_TEST"
#define VOLTAGE_POS_15 3.0f
#define VOLTAGE_NEG_15 0.75f
#define ADC_TO_VOLTAGE 0.000805861f //adc to voltage converstion

float deg_per_volt = 30 / (VOLTAGE_POS_15 - VOLTAGE_NEG_15);
float offset = 15 - (VOLTAGE_POS_15 * deg_per_volt); 

float angle adc_to_angle(int adc)
{
  //Convert adc to voltage 
  float voltage = adc * ADC_TO_VOLTAGE;

  //make sure voltage is in range 
  if (voltage > VOLTAGE_POS_15)
  {
    voltage = VOLTAGE_POS_15;
  }

  if (voltage < VOLTAGE_NEG_15)
  {
    voltage = VOLTAGE_NEG_15;
  }

  //convert voltage to angle 
  angle = voltage * deg_per_volt + offset; 

  return angle; 


}

//Calibration characteristics
esp_adc_cal_characteristics_t *adc_chars = NULL;

void app_main(void)
{

  const int dt_us = 1000;             //set sampling frequency 
  const float alpha = 0.2f;

  int adc_val;
  adc_oneshot_unit_handle_t adc_handle; 
  float angle_deg, angle_rad, last_angle_rad; 


  //Initialize ADC Oneshot Mode Driver on the ADC Unit 
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,                      //ADC1
      .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
  }; 

  adc_oneshot_new_unit(&init_config, &adc_handle);

  //Configure ADC Channel 7 (pin 35) w/ 12 bit resolution and 0-3.3 V 
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_12,                //12 bit resolution
      .atten = ADC_ATTEN_DB_11,                   //0-3.3 V 
  }; 

  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &config);

  //initialize filter adc value
  adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val); 
  float adc_filter = (float)adc_val; 

  //need to initialize all value compute angle etc.

  //initialze variable tracking the next update time 
  int64_t next_time = esp_timer_get_time();

  //Loop to read the ADC value 
  while (1)
  {

    //wait until next sample time 
    while(esp_timer_get_time() < next_time)
    {
      //do nothing 
    }

    //Read ADC Value 
    adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val);

    adc_filter = alpha * adc_val + (1.0f - alpha) * adc_filter;


    ESP_LOGI(TAG, "Raw: %d", adc_val);


    //Convert Voltage to angle
    //angle_deg = adc_to_angle(adc_filtered);

    //angle_deg to rad 

    //compute angular velocity

    //store the last angle value 
    //last_angle_rad = angle_rad;

    //Update the next sample time 
    next_time += dt_us;

  }
  

}
