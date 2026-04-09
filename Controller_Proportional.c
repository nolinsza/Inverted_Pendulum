#include <stdio.h>
#include <stdint.h>
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_adc/adc_oneshot.h>
#include "esp_log.h"
#include "esp_timer.h"  
#include "driver/ledc.h"
#include "esp_err.h"
#include <math.h>

#define TAG "ADC_TEST"
#define VOLTAGE_NEG_15 1.259f           
#define VOLTAGE_POS_15 2.423f


const int dt_us = 10000;             //set sampling frequency 
const float alpha = 1.0f;           //set alpha for digital filter (sampling adc)

//Parameters for controller 
const float kp = 162.0f;
const float kd = 10.0f;

int cnt = 0;

//const float deg_per_volt = 30.0f / (VOLTAGE_POS_15 - VOLTAGE_NEG_15);
//const float offset = 15 - deg_per_volt * VOLTAGE_POS_15;

float voltage;

float ADCtoVoltage(float x)
{
  float y = 0.000814f * x + 0.1369f;

  return y;
}

float VoltagetoAngle(float x)
{

    //make sure voltage is in range 
  if (x > VOLTAGE_POS_15)
  {
    x = VOLTAGE_POS_15;
  }

  if (x < VOLTAGE_NEG_15)
  {
    x = VOLTAGE_NEG_15;
  }

  float y = 25.76f * x - 47.291424f;

  return y;
}

void apply_right_fan(float input)
{
  if (input > 100.0f)
  {
    input = 100.0f;
  }

  int duty_cycle = (125.0f + input) * 1023.0f / 500.0f; 
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle); // Update the duty cycle
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); // Apply Right Fan
}

void apply_left_fan(float input)
{
    if (input > 100.0f)
  {
    input = 100.0f;
  }

  int duty_cycle = (125.0f + input) * 1023.0f / 500.0f; 
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty_cycle);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}


void app_main(void)
{

    vTaskDelay(pdMS_TO_TICKS(5000));

  //CODE FOR INITIALIZING THE ESCs

  //Define Struct for timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode       = LEDC_HIGH_SPEED_MODE, // Timer mode
      .timer_num        = LEDC_TIMER_0,        // Timer index
      .duty_resolution  = LEDC_TIMER_10_BIT,   // 10-bit resolution for duty cycle
      .freq_hz          = 2000,                // 2kHz frequency
      .clk_cfg          = LEDC_AUTO_CLK         // Auto select the source clock
  };

  
  int period_us = 500;
  int duty_counts = 250 * 1023 / period_us; // Convert pulse width to duty counts (10-bit resolution)

  //Configure Right Fan Channel
  ledc_channel_config_t ledc_channelR = {
    .gpio_num       = 18,                     // GPIO18 output
    .speed_mode     = LEDC_HIGH_SPEED_MODE,   // Channel mode
    .channel        = LEDC_CHANNEL_0,         // Channel index 
    .timer_sel      = LEDC_TIMER_0,          // Select the timer source
    .duty           = duty_counts,           // Initial pulse width (minimum ESC pulse)
    .hpoint         = 0                     // Start at beggining of the period 
  };

  //Configure Lef Fan Channel
  ledc_channel_config_t ledc_channelL = {
    .gpio_num       = 19,                     // GPIO18 output
    .speed_mode     = LEDC_HIGH_SPEED_MODE,   // Channel mode
    .channel        = LEDC_CHANNEL_1,         // Channel index 
    .timer_sel      = LEDC_TIMER_0,          // Select the timer source
    .duty           = duty_counts,           // Initial pulse width (minimum ESC pulse)
    .hpoint         = 0                     // Start at beggining of the period 
  };

  //Configure timer using struct 
  esp_err_t err = ledc_timer_config(&ledc_timer);

  //Configure channel using struct
  err = ledc_channel_config(&ledc_channelR);
  err = ledc_channel_config(&ledc_channelL);

  //Output max throttle for 10 seconds to calibrate ESC 
  vTaskDelay(pdMS_TO_TICKS(10000));

  //Output min throttle for 10 seconds to calibrate ESC
  int new_duty_counts = 125 * 1023 / 500; 
  //Update Right and Left Fan
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, new_duty_counts); // Update the duty cycle
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, new_duty_counts); 
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); // Apply Right Fan
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1); // Apply Left Fan
  vTaskDelay(pdMS_TO_TICKS(10000));

  //CODE FOR INITIALIZING THE ADC 
  int adc_val;
  adc_oneshot_unit_handle_t adc_handle; 
  float angle_deg, angle_rad, adc_filter;

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
  //1 ms waiting 
  const TickType_t period_ticks = pdMS_TO_TICKS(10);

  //initlaize waiting time
  TickType_t last_wake_time = xTaskGetTickCount();

  //initialize filter adc value
  adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val); 
  adc_filter = (float)adc_val;
  
  //intiialze the last_angle_rad
  //convert adc to angle
  voltage = ADCtoVoltage(adc_filter);
  angle_deg = VoltagetoAngle(voltage);
  float last_angle_rad = angle_deg * (M_PI/180.0f);

  ESP_LOGI(TAG, "Entering While Loop");

  //Loop to read the ADC value 
  while (1)
  {

     vTaskDelayUntil(&last_wake_time, period_ticks);

    //Read ADC Value 
    adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val);

    adc_filter = alpha * (float)adc_val + (1.0f - alpha) * adc_filter;

    //Convert Voltage to angle
    voltage = ADCtoVoltage(adc_filter);
   angle_deg = VoltagetoAngle(voltage);

    angle_rad = angle_deg * (M_PI/180.0f);

    float angular_vel = (angle_rad - last_angle_rad) / (dt_us * 1e-6);


    //compute the applied PWM
    float net_PWM = kp * angle_rad + kd * angular_vel; 

    //if net_PWM is negative apply force left 
    if (net_PWM < 0)
    {
      apply_left_fan(fabs(net_PWM) + 10);
      apply_right_fan(10);
    }
    else
    {
      apply_right_fan(fabs(net_PWM) + 10);
      apply_left_fan(10);
    }

    //ESP_LOGI(TAG, "Angle (deg): %f", angle_deg);
    //ESP_LOGI(TAG, "ADC VAL: %d", adc_val);
    //ESP_LOGI(TAG, "PWM_Output: %f", net_PWM);

    cnt++; 

    if (cnt == 100)
    {
      cnt = 0;
      ESP_LOGI(TAG, "Angle (deg): %f", angle_deg);
      ESP_LOGI(TAG, "Filtered ADC VAL: %f", adc_filter);
      ESP_LOGI(TAG, "Voltage: %f", voltage);
      ESP_LOGI(TAG, "Angular Vel: %f", angular_vel);
      ESP_LOGI(TAG, "PWM P: %f and D: %f", kp*angle_rad, kd*angular_vel);
      ESP_LOGI(TAG, "PWM_Output: %f", net_PWM);
    }

    last_angle_rad = angle_rad;

  }
  

}
