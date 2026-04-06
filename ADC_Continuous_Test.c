

//Callback function for continuous ADC 
static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t haqndle, const adc_continuous_evt_data_t *edata, void *user_data) {
    // Perform light processing, e.g., notify a task
    // DO NOT use blocking code (like printf or vTaskDelay) here
    return false; // return true if a high priority task was woken up
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
        .resolution_hz = 1000000, // 1 MHz = 1 tick = 1 us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,  // 1000 us = 1 kHz
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
        .max_store_buf_size = 1024,            //storage size in bytes
        .conv_frame_size = 256,                //define the number of bytes to process at one time (frame)
    };

  adc_continuous_new_handle(&adc_config, &handle);

  adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 100000,              //set the sampling frequency - 100kHz
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
  adc_continuous_register_event_callbacks(handle, &cbs, NULL)

  //start the ADC sampling
  adc_continuous_start(handle)

    // Create control task
    xTaskCreate(control_task, "control_task", 4096, NULL, 10, &control_task_handle);

    // Init hardware timer
    init_timer();
}
