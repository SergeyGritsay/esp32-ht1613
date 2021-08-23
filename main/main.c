#include <stdio.h>
#include <math.h>
#include <esp32/rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "esp_log.h"

#define data_LED GPIO_NUM_17
//uint8_t data = 0b0001;

void set_1MHz_clock_on_GPIO18(void)
{
	gpio_config_t io_conf = {
		.intr_type    = GPIO_PIN_INTR_DISABLE,
		.mode         = GPIO_MODE_INPUT_OUTPUT_OD,
		.pin_bit_mask = ( 1ULL << GPIO_NUM_18 ),
		.pull_down_en = 0,
		.pull_up_en   = 0,
	};
   
	gpio_config(&io_conf);
	
	periph_module_enable(PERIPH_LEDC_MODULE);

    // Set up timer
	ledc_timer_config_t ledc_timer = {
    
		.duty_resolution = LEDC_TIMER_1_BIT,

    	.freq_hz = 1000000,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = LEDC_TIMER_0
    };

	ledc_timer_config(&ledc_timer); // Set up GPIO PIN 
	   
	ledc_channel_config_t channel_config = {
		.channel    = LEDC_CHANNEL_0,
		.duty       = 1,	
		.gpio_num   = 18,                        // GPIO pin
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_sel  = LEDC_TIMER_0 
	};
	
	ledc_channel_config(&channel_config);
}

void set_low_level(int gpio_output)
{
	gpio_set_direction(gpio_output, GPIO_MODE_INPUT_OUTPUT);
	int clock_level = 0;
	gpio_set_level(gpio_output, clock_level);
	vTaskDelay(200 / portTICK_PERIOD_MS);
	clock_level = gpio_get_level(gpio_output);
	//printf("level: %d\n",clock_level);
}

void set_high_level(int gpio_output)
{
	gpio_set_direction(gpio_output, GPIO_MODE_INPUT_OUTPUT);
	int clock_level = 1;
	gpio_set_level(gpio_output, clock_level);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	clock_level = gpio_get_level(gpio_output);	
	//printf("level: %d\n", clock_level);
}

void ht1613_send_byte(uint8_t byte)
{
	gpio_set_direction(data_LED, GPIO_MODE_INPUT_OUTPUT);
	for (uint8_t i = 0; i < 4; i++ )
	{
		int data_level = byte & 0b1000;
		gpio_set_level(GPIO_NUM_17, data_level);
		data_level = gpio_get_level(data_LED);
		printf("level data: %d\n", data_level);
		vTaskDelay(300 / portTICK_PERIOD_MS);
		byte <<= 1; // 0b0001 -> 0b0010 -> 0b0100 -> 0b1000 D3:0, D2:0, D1:0, D0:1 
	};
}

void app_main(void)
{
	uint8_t data = 0b0001;
	gpio_config_t io_conf = {
		.intr_type    = GPIO_PIN_INTR_DISABLE,
		.mode         = GPIO_MODE_INPUT_OUTPUT_OD,
		.pin_bit_mask = ( 1ULL << data_LED ),
		.pull_down_en = 0,
		.pull_up_en   = 0,
	};
    gpio_config(&io_conf);
	set_1MHz_clock_on_GPIO18();
	// Main loop
    while(1)
	{
		if((gpio_get_level(GPIO_NUM_18) == 0) && (data <= 0b1010))
		{
			
			// on 1->0 transfer i think
			printf("data: %d\n", data);
			ht1613_send_byte(data);
			set_high_level(GPIO_NUM_18);
			data += 0b0001;
			
		} else
		{
			if ((data <= 0b1010) && (gpio_get_level(GPIO_NUM_18 == 1)))
			{
				set_low_level(GPIO_NUM_18);
			}
			else
			{
				set_low_level(GPIO_NUM_18);
				break;
			};
		};
	};
}