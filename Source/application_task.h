#include "gd32f30x.h"

extern uint8_t led_mode;
extern uint8_t timer_mode;
extern uint32_t counter;
extern uint32_t alarm;
void led_task_welcome(void *pvParameters);
void neoway_task_init(void *pvParameters);
void led_task_alarm(void* pvParameters);
void btn_task(void *pvParameters);
void neoway_task_subsribe(void* pvParameters);

void at24c256_task(void *pvParameters);
void w25q128_task(void *pvParameters);