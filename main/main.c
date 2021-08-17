#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "esp_log.h"
#include "time.h"

#define data_LED 17


void set_1MHz_clock_on_GPIO18(void)
{
	gpio_config_t io_conf = {
		.intr_type    = GPIO_PIN_INTR_DISABLE,
		.mode         = GPIO_MODE_INPUT_OUTPUT_OD,
		.pin_bit_mask = ( 1ULL << GPIO_NUM_18),
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
};

/* data example 
	0b0000, 0
	0b0001, 1
	0b0010, 2
	0b0011, 3
	0b0100, 4
	0b0101, 5
	0b0110, 6
	0b0111, 7
	0b1000, 8
	0b1001  9
*/

void ht1613_task(void * arg)
{
	
}
void set_level(int gpio_output){
	gpio_set_direction(gpio_output, GPIO_MODE_INPUT_OUTPUT);
	int clock_level = gpio_get_level(gpio_output);
	gpio_set_level(gpio_output, !clock_level);
	clock_level = gpio_get_level(gpio_output);
	//printf("%d\n",clock_level);
	//vTaskDelay(1000/portTICK_PERIOD_MS);
}

void app_main(void){
	//int time = 0;
	//int time_1 = 0;
	uint8_t data = 0b0000;
	gpio_config_t io_conf = {
		.intr_type    = GPIO_PIN_INTR_DISABLE,
		.mode         = GPIO_MODE_INPUT_OUTPUT_OD,
		.pin_bit_mask = (1ULL << data_LED),
		.pull_down_en = 0,
		.pull_up_en   = 0,
	};
    gpio_config(&io_conf);
	set_1MHz_clock_on_GPIO18();
	//set_level(GPIO_NUM_18);
	//set_level(GPIO_NUM_18);
	// Main loop
    while(1) {
		//printf("%d\n", clock_level);
		if ((gpio_get_level(GPIO_NUM_18) == 0) && (data <= 0b1001)){
			//set_level(GPIO_NUM_18);
			printf("%d\n",data);
			set_level(GPIO_NUM_18);
			data += 1;
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		} else {if (data<= 0b1001){
			//printf("thats all\n");//?????????????????????
			set_level(GPIO_NUM_18);
			vTaskDelay(1000/ portTICK_PERIOD_MS);
			//break;
		} else {
		break;
		};};
		};	
} 
