#include <stdio.h>
#include <stdint.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_adc/adc_oneshot.h>
#include "esp_log.h"

void app_main(void)
{

  int adc_val;
  adc_oneshot_unit_handle_t adc_handle; 
  int32_t angle_val; 


  //Initialize ADC Oneshot Mode Driver on the ADC Unit 
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,                      //ADC1
      .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
  }; 

  adc_oneshot_new_unit(&init_config, &adc_handle);

  //Configure ADC Channel 7 w/ 12 bit resolution and 0-3.3 V 
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_12,                //12 bit resolution
      .atten = ADC_ATTEN_DB_11,                   //0-3.3 V 
  }; 

  adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &config);

  //Loop to read the ADC value 
  while (1)
  {
    //Read ADC Value 
    adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val);
    //Print ADC Value 
    ESP_LOGI("ADC Value", "%d", adc_val);

    //Convert Voltage to angle - *May want to adjust this for a range of say -5 to 5 degrees for stronger resolution for actualy implementation 
    if (adc_val > 4060) //maximum range 
    {
      printf("above max range");
    }
    else if (adc_val > 2668)  //compute positive angle 
    {
      angle_val = (adc_val - 2668) * 5407 / 1000000;
    }
    else if (adc_val < 1210)  //compute negative angle 
    {
      angle_val = (adc_val - 2668) * 5169/100000; 
    }
    else
    {
      printf("below min range");
    }

    //Delay 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
  

}