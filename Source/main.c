/*

Demo Description

Press button 1 to set the timer for reminder
Press button 2 to select the LED alarm mode


timer_mode=1: 15 second
timer_mode=2: 1/2 minute
timer_mode=3: 1 minute
timer_mode=4: 2 minute

led_mode=1: running led
led_mode=2: flash red 500 ms
led_mode=3: flash green 500 ms
led_mode=4: flash blue 500 ms
*/


#include "header_files.h"

uint8_t led_mode=1;
uint8_t timer_mode=1;
uint32_t counter=0;
uint32_t alarm=15;

static void HardWareInit(void){
	systick_config();
	atc25_led_init();
	spi_config();
	i2c_config();
}


int main(void)
{
	HardWareInit();
	
	xTaskCreate(led_task_welcome, "Led_task_welcome", configMINIMAL_STACK_SIZE+200, NULL, 4, NULL);
	xTaskCreate(neoway_task_init, "neoway_task_init", configMINIMAL_STACK_SIZE+200, NULL, 3, NULL);
	xTaskCreate(btn_task, "btn_task", configMINIMAL_STACK_SIZE+200, NULL, 2, NULL);
	xTaskCreate(led_task_alarm, "led_task_alarm", configMINIMAL_STACK_SIZE+200, NULL, 2, NULL);
	
	vTaskStartScheduler();
	while(1){}
}

uint32_t tickhook_cnt = 0;


void vApplicationTickHook(void) {
	tickhook_cnt++;
}
