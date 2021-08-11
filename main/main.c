#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"

#define data_LED 17


void set_1MHz_clock_on_GPIO2(void)
{
   periph_module_enable(PERIPH_LEDC_MODULE);

   // Set up timer
   ledc_timer_config_t ledc_timer = {
     // We need clock, not PWM so 1 bit is enough.
     .duty_resolution = LEDC_TIMER_1_BIT,

     // Clock frequency, 1 MHz, high speed
     .freq_hz = 10000000,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .timer_num = LEDC_TIMER_0,

     // I think not needed for new esp-idf software, try uncommenting
     .clk_cfg = LEDC_USE_APB_CLK
   };

   ledc_timer_config(&ledc_timer); // Set up GPIO PIN 

   ledc_channel_config_t channel_config = {
     .channel    = LEDC_CHANNEL_0,
     .duty       = 1,
     .gpio_num   = 18,                        // GPIO pin
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .timer_sel  = LEDC_TIMER_0 
   };

   ledc_channel_config(&channel_config);
}
void app_main(void){
	
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT_OD;
	io_conf.pin_bit_mask = (1ULL << data_LED);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	//set_1MHz_clock_on_GPIO2();
	set_1MHz_clock_on_GPIO2();
	int clock_level = gpio_get_level(GPIO_NUM_18);
	printf("%d\n",clock_level);
    // Main loop
    while(1) {
		/* set_1MHz_clock_on_GPIO2();*/
		int clock_level = gpio_get_level(GPIO_NUM_2);
		printf("%d\n",clock_level);
		/*         gpio_set_level(clk_LED, 0);
		printf("___\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(clk_LED, 1);
		printf("---\n");
        vTaskDelay(1000 / portTICK_RATE_MS); */
  }
	printf("\n");
} 
