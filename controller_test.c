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
#define VOLTAGE_NEG_15 2705           
#define VOLTAGE_POS_15 4095
#define VOLTAGE_0 3226
#define INTEGRAL_MAX 0.2f               //do not let the integral of the error get larger than this value
#define INTEGRAL_CHANGE_THRESHOLD 0.01f //begin decreasing the integral if the error is less than this value
#define INTEGRAL_SCALE_FACTOR 0.75f     //the factor to decrease the integral by 

const int dt_us = 1000;             //set sampling frequency 
const float alpha = 0.2f;           //set alpha for digital filter (sampling adc)
const float beta = 0.2f;            //set beta for digital filter (angular velocity)

const float a = 0.000197;            //paramter a for quadratic force -> thrust curve
const float b = 0.003229;            //paramber b for qudratic force .> thrust curve


//Parameters for controller 
const float g1 = -3.8f;             //gain of angle 
const float g2 = -0.35f;               //gain of angular velocity
const float g3 = 0.3f;              //gain of integral of error 

const float deg_per_adc = 15.0f / (VOLTAGE_0 - VOLTAGE_NEG_15);

void apply_right_fan(int input)
{
  if (input > 100)
  {
    input = 100;
  }

  int duty_cycle = (125 + input) * 1023 / 500; 
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle); // Update the duty cycle
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0); // Apply Right Fan
}

void apply_left_fan(int input)
{
    if (input > 100)
  {
    input = 100;
  }

  int duty_cycle = (125 + input) * 1023 / 500; 
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty_cycle);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

float adc_to_angle(int adc)
{

  //make sure voltage is in range 
  if (adc > VOLTAGE_POS_15)
  {
    adc = VOLTAGE_POS_15;
  }

  if (adc < VOLTAGE_NEG_15)
  {
    adc = VOLTAGE_NEG_15;
  }

  //convert voltage to angle 
  float angle = (adc - VOLTAGE_0) * deg_per_adc; 

  return angle; 


}


void app_main(void)
{

  //Initialize variable 
  float error = 0;
  float int_error = 0;
  float last_int_error = 0;
  float force_req;

  //CODE FOR INITIALIZING THE ESCs

  //Delay 10 seconds before doing anything 
  vTaskDelay(pdMS_TO_TICKS(10000));

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
  float angle_deg, angle_rad, angle_filter, last_angle_filter, angular_vel; 

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

  //set next update time 
  int64_t next_time = esp_timer_get_time() + dt_us;

  //initialize filter adc value
  adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val); 
  float adc_filter = (float)adc_val;
  
  //intiialze the last_angle_rad
  //convert adc to angle
  angle_deg = adc_to_angle(adc_filter);

  //angle_deg to rad 
  angle_filter = angle_deg * (M_PI/180.0f);

  last_angle_filter = angle_filter;

  //Loop to read the ADC value 
  while (1)
  {

    //wait until next sample time 
    while(esp_timer_get_time() < next_time)
    {
      //very short delay to avoid watchdog timeout 
      vTaskDelay(pdMS_TO_TICKS(dt_us / 1000));  
    }

    //Update the next sample time 
    next_time += dt_us;

    //Read ADC Value 
    adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_val);

    adc_filter = alpha * (float)adc_val + (1.0f - alpha) * adc_filter;

    //Convert Voltage to angle
    angle_deg = adc_to_angle(adc_filter);

    ESP_LOGI(TAG, "Angle (deg): %f", angle_deg);
    ESP_LOGI(TAG, "ADC VAL: %d", adc_val);

    /*
    if(angle_deg > 3.0f)
    {
      //turn on left fan
        apply_left_fan(30);
        apply_right_fan(10);
        int_error = 0;
        adc_filter = adc_val;
        last_angle_rad = angle_deg * (M_PI/180.0f);
        continue;
    }
    else if (angle_deg <-3.0f)
    {
      //turn on right fan 
      apply_right_fan(30);
      apply_left_fan(10);
      int_error = 0;
      adc_filter = adc_val;
      last_angle_rad = angle_deg * (M_PI/180.0f);
      continue;
    }
    */

    angle_rad = angle_deg * (M_PI/180.0f);

    //Filter for angular velocity
    angle_filter = beta * angle_rad + (1.0f -beta) * angle_filter;
    //compute angular velocity
    angular_vel = (angle_filter - last_angle_filter) / (dt_us * 1e-6f);

    //Compute the required force using the state space controller
    //compute the error
    error = 0 - angle_rad; 
    //integral of the error 
    int_error += error * dt_us * 1e-6f;

    if (fabsf(angle_rad) < INTEGRAL_CHANGE_THRESHOLD)
    {
      int_error = INTEGRAL_SCALE_FACTOR * int_error; 
    }

    //do not let integral get too large 
    if (int_error > INTEGRAL_MAX)
    {
      int_error = INTEGRAL_MAX; 
    }
    else if (int_error <-INTEGRAL_MAX)
    {
      int_error = -INTEGRAL_MAX;
    }

    //Required Force Output 
    force_req = -g1*angle_rad - g2*angular_vel -g3*int_error;

    //Compute the thrust output using the required force
    int thrust_input = (-1 * b + sqrtf(b*b + 4 * a * fabsf(force_req))) / (2 * a); 
    if (force_req > 0)
    {
        //turn on left fan 
        apply_left_fan(thrust_input);
        apply_right_fan(3);
    }
    else 
    {
        //turn on right fan
        apply_right_fan(thrust_input);
        apply_left_fan(3);
    }

    ESP_LOGI(TAG, "ADC Filtered: %f", adc_filter);
    ESP_LOGI(TAG, "Angular Velocity: %f", angular_vel);
    ESP_LOGI(TAG, "Integral Error Val: %f", int_error);
    ESP_LOGI(TAG, "Force Required: %f", force_req);
    ESP_LOGI(TAG, "Thrust Output %d", thrust_input);

    last_int_error = int_error; 
    last_angle_filter = angle_filter;

  }
  

}