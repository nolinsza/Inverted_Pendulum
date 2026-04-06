
#define OVERSAMPLE_RATE 512 //Use the last 512 ADC values

const int dt_us = 1000;

uint16_t adc_buffer[OVERSAMPLE_RATE] = {0};
uint32_t adc_sum = 0;
uint16_t adc_average = 0;
uint16_t index = 0;

//Callback function for continuous ADC 
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {

    //create a pointer to the buffer
    uint8_t *buf = edata->conv_frame_buffer;   //point to the adc data 
    uint32_t len = edata->size;                //number of bytes in the buffer

    for (int i = 0; i<len; i+=2)                //2 bytes per sample 
        {

            //extract adc sample 
            uint16_t sample = ((uint16_t)buf[i] | (buf[i+1] << 8)) & 0x0FFF;
            
            //Remove oldest value
            adc_sum -= adc_buffer[index];

            //Store new sample
            adc_buffer[index] = sample;

            //add the new sample 
            adc_sum += sample;

            //update the index 
            index = (index + 1) & (OVERSAMPLE_RATE -1); //raps around same thing as (index + 1) % 512
        }

    //compute average
    adc_average = adc_sum / OVERSAMPLE_RATE; 
    
}

//Callback for a timer to run at 1 KHz 
static bool IRAM_ATTR timer_isr_callback(void *args)
{
    BaseType_t high_task_wakeup = pdFALSE;

    vTaskNotifyGiveFromISR(control_task_handle, &high_task_wakeup);

    return (high_task_wakeup == pdTRUE);
}

//Function to set up the 1kHz timer 
gptimer_handle_t timer = NULL;

void init_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = dt_us, // 1 MHz -> 1 tick = 1 us
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

//set up a task handler for loop timing 
static TaskHandle_t control_task_handle = NULL;
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


void app_main(void)
{
  //Configure continuous ADC initialization
  adc_continuous_handle_t handle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 512,            //storage size in bytes
        .conv_frame_size = 512,                //define the number of bytes to process at one time (frame)
    };

  adc_continuous_new_handle(&adc_config, &handle);

  adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1000000,              //set the sampling frequency - 1MHz
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

  //start the ADC sampling
  adc_continuous_start(handle);

    // Create control task
    xTaskCreate(control_task, "control_task", 4096, NULL, 10, &control_task_handle);

    // Init hardware timer
    init_timer();
}
