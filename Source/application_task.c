#include "application_task.h"
#include "atc_e25.h"
#include "freeRtos.h"
#include "task.h"
#include "semphr.h"

void led_task_welcome(void* parameters)
{
	for(;;)
	{
		atc25_led_running(200);
		uart3_init();
		nvic_irq_enable(UART3_IRQn,0,0);
		usart_interrupt_enable(UART3,USART_INT_RBNE);
		usart_string_transmit(UART3, (uint8_t*)"AT\r\n");
		vTaskDelete(NULL);
	}
}
void neoway_task_init(void* parameters)
{
	for(;;)
	{
		neoway_init();
		vTaskDelete(NULL);
	}
}
void btn_task(void* parameters)
{
	for(;;)
	{
		if(gpio_input_bit_get(BTN1_GPIO_PORT, BTN1_PIN)==RESET)
		{
			while(gpio_input_bit_get(BTN1_GPIO_PORT, BTN1_PIN)==RESET);
			led_mode++;
			switch(led_mode)
			{
				case 1:
					atc25_led_flash(RED, 500);
					break;
				case 2:
					atc25_led_flash(GREEN, 500);
					break;
				case 3:
					atc25_led_flash(BLUE, 500);
					break;
				case 4:
					atc25_led_running(200);
					break;
				case 5:
					led_mode = 1;
					atc25_led_flash(RED, 500);
					break;
				default:
					atc25_led_off(ALL);
		}
	}
		if(gpio_input_bit_get(BTN2_GPIO_PORT, BTN2_PIN)==RESET)
		{
			while(gpio_input_bit_get(BTN2_GPIO_PORT, BTN2_PIN)==RESET);
			timer_mode++;
			switch(timer_mode)
			{
				case 1:
					atc25_led_on(RED);
					vTaskDelay(pdMS_TO_TICKS(500));
					atc25_led_off(RED);
					alarm = 15;
					break;
				case 2:
					atc25_led_on(GREEN);
					vTaskDelay(pdMS_TO_TICKS(500));
					atc25_led_off(GREEN);
					alarm = 30;
					break;
				case 3:
					atc25_led_on(BLUE);
					vTaskDelay(pdMS_TO_TICKS(500));
					atc25_led_off(BLUE);
					alarm = 60;
					break;
				case 4:
					atc25_led_on(ALL);
					vTaskDelay(pdMS_TO_TICKS(500));
					atc25_led_off(ALL);
					alarm = 120;
					break;
				case 5:
					timer_mode=1;
					atc25_led_on(RED);
					vTaskDelay(pdMS_TO_TICKS(500));
					atc25_led_off(RED);
					alarm = 15;
					break;
				default:
					atc25_led_off(ALL);
			}		
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
void led_task_alarm(void* pvParameters)
{
	for(;;)
	{
		if(++counter>alarm)
		{
			counter = 0;
			switch(led_mode)
			{
				uint8_t i;
				case 1:
					for(i=0; i<5; i++)
					atc25_led_flash(RED, 500);
					break;
				case 2:
					for(i=0; i<5; i++)
					atc25_led_flash(GREEN, 500);
					break;
				case 3:
					for(i=0; i<5; i++)
					atc25_led_flash(BLUE, 500);
					break;
				case 4:
					for(i=0; i<5; i++)
					atc25_led_running(200);
					break;
				default:
					break;
			}
			neoway_pub(0, 1, "lixytopic", "xin chao...");
			neoway_subscribe("lixytopic", 1);
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
