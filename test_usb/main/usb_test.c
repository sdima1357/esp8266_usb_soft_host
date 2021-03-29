/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <sys/time.h>
#include "esp8266/eagle_soc.h"
#include "esp8266/pin_mux_register.h"
#include "esp8266/gpio_struct.h"
#include "esp_clk.h"
#include "driver/gpio.h"

#define DP_P  12
#define DM_P  14
#define DP_P1  15
#define DM_P1  13
#define DP_P2  -1
#define DM_P2  -1
#define DP_P3  -1
#define DM_P3  -1

void led(int on_fff)
{
	//		gpio_set_level(BLINK_GPIO, on_fff);
}



#include "usb_host.h"
#define ENTER_CRITICAL() portENTER_CRITICAL()
#define EXIT_CRITICAL() portEXIT_CRITICAL()
#include "driver/hw_timer.h"


void hw_timer_callback1ms()
{
	portENTER_CRITICAL();		
	usb_process();
	portEXIT_CRITICAL();
}
void app_main()
{
    //~ gpio_pad_select_gpio(DP_P);
    //~ gpio_set_direction(DP_P, GPIO_MODE_OUTPUT);
	
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("esp_get_free_heap_size() %d\n",esp_get_free_heap_size());
    printf("esp_get_minimum_free_heap_size() %d\n",esp_get_minimum_free_heap_size());
	
    //system_get_chip_id();
				
				uint32_t freq = esp_clk_cpu_freq()/1000000;
				printf("cpu freq = %d MHz\n",freq);
	    
  //  int freq = system_get_cpu_freq();	
    printf("This is ESP8266 chip with %d CPU cores, WiFi, ",
            chip_info.cores);

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
     initStates(DP_P,DM_P,DP_P1,DM_P1,DP_P2,DM_P2,DP_P3,DM_P3);
     
    hw_timer_init(hw_timer_callback1ms, NULL);
    hw_timer_alarm_us(1000, true);
    
    while(1)
    {
	    printState();
	    vTaskDelay(10 / portTICK_PERIOD_MS);
     };
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
