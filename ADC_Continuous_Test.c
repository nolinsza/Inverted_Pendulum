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
#include "esp_adc/adc_continuous.h"
#include "driver/gptimer.h"


#define TAG "ADC_TEST"
#define VOLTAGE_NEG_15 2000
#define VOLTAGE_POS_15 3000
#define OVERSAMPLE_RATE 256 //Use the last 256 ADC values and break this into 2 256 byte operations

const int dt_us = 1000;

//Parameters for controller 
const float kp = 162.0f;
const float kd = 10.0f;

//Global variables for the control loop 
int cnt = 0;
float angle_deg, angle_rad, last_angle_rad;

//variables for the ADC Callback
uint16_t adc_buffer[OVERSAMPLE_RATE] = {0};
uint32_t adc_sum = 0;
volatile uint16_t adc_average = 0;
uint16_t idx = 0;

  adc_continuous_handle_t handle = NULL;
  static TaskHandle_t control_task_handle = NULL;
  static TaskHandle_t adc_task_handle = NULL;

  void control_loop(void);

//--------------------------------------- CONTINUOUS ADC CALLBACK FUNCTION ------------------------------------------------
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {

    //Notify ADC Task to read DMA and average ADC 
    BaseType_t high_task_wakeup = pdFALSE;
    vTaskNotifyGiveFromISR(adc_task_handle, &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
    
}

//--------------------------------------- AVERAGE THE ADC DATA --------------------------------------------------------------------
void adc_task(void *arg)
{
    uint8_t buffer[OVERSAMPLE_RATE];
    uint32_t length = 0;

    while (1)
    {
        //Wait for notfication from ADC ISR 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //Load the data and lngeth 
        while (adc_continuous_read(handle, buffer, sizeof(buffer), &length, 0) == ESP_OK)
        {
            {
                for (int i = 0; i<length; i+=2)                //2 bytes per sample 
                {

                    //extract adc sample 
                    uint16_t sample = ((uint16_t)buffer[i] | (buffer[i+1] << 8)) & 0x0FFF;
                    
                    //Remove oldest value
                    adc_sum -= adc_buffer[idx];

                    //Store new sample
                    adc_buffer[idx] = sample;

                    //add the new sample 
                    adc_sum += sample;

                    //update the index 
                    idx = (idx + 1) & (OVERSAMPLE_RATE -1); //wraps around same thing as (index + 1) % OVERSAMPLE_RATE
                }

                adc_average = adc_sum >> 8; //same as dividing by the OVERSAMPLE_RATE
            }
        }

    }
}

//----------------- CALLBACK FOR CONTROL LOOP TIMER -----------------------------------------------
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    vTaskNotifyGiveFromISR(control_task_handle, &high_task_wakeup);

    return (high_task_wakeup == pdTRUE);
}

//------------------------------- FUNCTION TO SET UP CONTROL LOOP TIMER -------------------------------------
gptimer_handle_t timer = NULL;
void init_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   //1 MHz resolution -> 1 tick = 1 us 
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = dt_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

//------------------------ CONTROL TASK THAT CALLS THE CONTROL FUNCTION ----------------------------
void control_task(void *arg)
{
    while (1)
    {
        //Wait for timer interrupt
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //Controller loop 
        control_loop();
    }
}

//------------------------------- ADC TO ANGLE FORMULA -------------------------------
float ADCtoAngle(int x)
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

  float y = 0.004f * x - 47.291424f;

  return y;
}


// -------------- FUNCTION TO APPLY OUTPUTS TO THE MOTOR -------------------------
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

//------------------------------------------ CONTROL LOOP FUNCTION ---------------------------------------
void control_loop(void)
{
    //Copy the ADC Value 
    uint16_t adc_copy = adc_average; 

    //Convert the ADC to an angle 
    angle_deg = ADCtoAngle(adc_copy);
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

        cnt++; 

    if (cnt == 1000)
    {
      cnt = 0;

      ESP_LOGI(TAG, "ADC VAL: %d", adc_copy);
      ESP_LOGI(TAG, "Angle (deg): %f", angle_deg);
      ESP_LOGI(TAG, "Angular Vel: %f", angular_vel);
      ESP_LOGI(TAG, "PWM P: %f and D: %f", kp*angle_rad, kd*angular_vel);
      ESP_LOGI(TAG, "PWM_Output: %f", net_PWM);
    }

    last_angle_rad = angle_rad;
}

// ----------------------------------------- MAIN FUNCTION ------------------------------------------------
void app_main(void)
{
  //----------------------------------------- CODE FOR CONTINUOUS ADC ------------------------------------

  adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 512,            //storage size in bytes
        .conv_frame_size = 256,                //define the number of bytes to process at one time (frame)
    };

  adc_continuous_new_handle(&adc_config, &handle);

  adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 270000,              //set the sampling frequency - 270 KHz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,   //unit 1 
    };

  //configure pattern for dig_cfg
   adc_digi_pattern_config_t adc_pattern[1] = {0};
    dig_cfg.pattern_num = 1;                   //one ADC unit 
    adc_pattern[0].atten = ADC_ATTEN_DB_11;    //0-3.3 V
    adc_pattern[0].channel = ADC_CHANNEL_7;    //GPIO 35
    adc_pattern[0].unit = ADC_UNIT_1;    
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;  //12 bits per sample

    dig_cfg.adc_pattern = adc_pattern;

    adc_continuous_config(handle, &dig_cfg);

  //trigger ISR when frame is done processing 
  adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
  };
  adc_continuous_register_event_callbacks(handle, &cbs, NULL);

  //Create task for processing the ADC data 
  xTaskCreate(adc_task, "adc_task", 4096, NULL, 9, &adc_task_handle);

  //start the ADC sampling
  adc_continuous_start(handle);


  //---------------------- CODE FOR INITIALIZING THE ESCs -----------------------------------------

  vTaskDelay(pdMS_TO_TICKS(5000));

  //Define Struct for timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode       = LEDC_HIGH_SPEED_MODE, // Timer mode
      .timer_num        = LEDC_TIMER_0,        // Timer index
      .duty_resolution  = LEDC_TIMER_10_BIT,   // 10-bit resolution for duty cycle
      .freq_hz          = 2000,              // 2kHz frequency
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
  
  // ----------------------- Initialize variables and start the control loop timer  ---------------------------------
  
  // Create control task
  xTaskCreate(control_task, "control_task", 4096, NULL, 10, &control_task_handle);

  //Initialize the last_angle_rag value
  int ADC = adc_average;
  angle_deg = ADCtoAngle(ADC);
  last_angle_rad = angle_deg * (M_PI / 180.0f);
  
  //Init hardware timer to trigger the control loop 
  init_timer();

}
