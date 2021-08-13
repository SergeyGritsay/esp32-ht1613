#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"

#define data_LED 17


void set_1MHz_clock_on_GPIO18(void)
{
	periph_module_enable(PERIPH_LEDC_MODULE);

   // Set up timer
	ledc_timer_config_t ledc_timer = {
     // We need clock, not PWM so 1 bit is enough.
		.duty_resolution = LEDC_TIMER_1_BIT,

     // Clock frequency, 1 MHz, high speed
		.freq_hz = 1000000,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,

     // I think not needed for new esp-idf software, try uncommenting
		.clk_cfg = LEDC_USE_APB_CLK
	};

	ledc_timer_config(&ledc_timer); // Set up GPIO PIN 

	gpio_config_t io_conf = {
		.intr_type       = GPIO_PIN_INTR_DISABLE,
		.mode         = GPIO_MODE_INPUT_OUTPUT_OD,
		.pin_bit_mask = ( 1ULL << GPIO_NUM_18),
		.pull_down_en = 0,
		.pull_up_en   = 0,
	};
   
	gpio_config(&io_conf);
   
	ledc_channel_config_t channel_config = {
		.channel    = LEDC_CHANNEL_0,
		.duty       = 1,
		.gpio_num   = 18,                        // GPIO pin
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_sel  = LEDC_TIMER_0 
	};
	
	ledc_channel_config(&channel_config);
};
const uint8_t data_set [10]={
	0x0000,
	0x0001,
	0x0010,
	0x0011,
	0x0100,
	0x0101,
	0x0110,
	0x0111,
	0x1000,
	0x1001
};

void app_main(void){
	//_74hc595_test();
	//int data = 0x0000;
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
	io_conf.pin_bit_mask = (1ULL << data_LED);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	//set_1MHz_clock_on_GPIO2();
	set_1MHz_clock_on_GPIO18();
	int clock_level = gpio_get_level(GPIO_NUM_18);
	//printf("%d\n",clock_level);
    // Main loop
	int i=0;
    while(1) {
		gpio_set_level(GPIO_NUM_18, 1);
		clock_level = gpio_get_level(GPIO_NUM_18);
		if ((clock_level == 0) && (i != 10)){
			printf("%d\n",data_set[i]);
			i +=1;
			vTaskDelay(100);
		} else{
			printf("thats all\n");
			break;
		}
		gpio_set_level(GPIO_NUM_18, 1);
		//printf("%d\n", data);
	};
	//printf("\n");
} 
