//this header file define ATC-Board 25 hardware

#include "gd32f30x.h"

typedef enum 
{
    KEY_MODE_GPIO = 0,
    KEY_MODE_EXTI = 1
} keymode_typedef_enum;

typedef enum 
{
    RED = 0,
    GREEN = 1,
		BLUE = 2,
		ALL =3
} ledlist_typedef_enum;

#define AT24C256_ADDR 0xA0

//define user RGB LED on board
#define LED_GREEN_PORT				GPIOC
#define LED_GREEN_PIN					GPIO_PIN_9
#define LED_GREEN_CLK					RCU_GPIOC

#define LED_BLUE_PORT					GPIOA
#define LED_BLUE_PIN					GPIO_PIN_8
#define LED_BLUE_CLK					RCU_GPIOA

#define LED_RED_PORT					GPIOC
#define LED_RED_PIN						GPIO_PIN_8
#define LED_RED_CLK						RCU_GPIOC



// BTN1(PA15) 
#define BTN1_PIN                    GPIO_PIN_15
#define BTN1_GPIO_PORT              GPIOA
#define BTN1_GPIO_CLK               RCU_GPIOA
#define BTN1_EXTI_LINE              EXTI_15
#define BTN1_EXTI_PORT_SOURCE       GPIO_PORT_SOURCE_GPIOA
#define BTN1_EXTI_PIN_SOURCE        GPIO_PIN_SOURCE_15
#define BTN1_EXTI_IRQn              EXTI10_15_IRQn

// BTN2 (PC12) 
#define BTN2_PIN                    GPIO_PIN_12
#define BTN2_GPIO_PORT              GPIOC
#define BTN2_GPIO_CLK               RCU_GPIOC
#define BTN2_EXTI_LINE              EXTI_12
#define BTN2_EXTI_PORT_SOURCE       GPIO_PORT_SOURCE_GPIOC
#define BTN2_EXTI_PIN_SOURCE        GPIO_PIN_SOURCE_12
#define BTN2_EXTI_IRQn              EXTI10_15_IRQn


void atc25_led_init(void);
void atc25_led_on(ledlist_typedef_enum led_num);
void atc25_led_off(ledlist_typedef_enum led_num);
void atc25_led_running(uint32_t time_delay_ms);
void atc25_btn1_init(keymode_typedef_enum btn_mode);
void atc25_btn2_init(keymode_typedef_enum btn_mode);
void usart0_init(void);
void uart3_init(void);
void atc25_led_flash(ledlist_typedef_enum led_num, uint32_t time_ms);


void usart_string_transmit(uint32_t usart_periph, uint8_t data[]);
void UART3_IRQHandler(void);
bool atc25_check_string(char* str);
void neoway_init(void);
void neoway_set_param(char* clientID, char* username, char* password);
void neoway_connect(char* host, uint8_t clean, uint8_t keep_alive);
void neoway_subscribe(char* topic_name, uint8_t qos);
void neoway_pub(uint8_t retained, uint8_t qos, char* topicname, char* message);


void i2c_config(void);
void eeprom_write(uint32_t i2c_periph, uint16_t addr, uint8_t data);
uint8_t eeprom_read(uint32_t i2c_periph, uint16_t addr);


void spi_config(void);
void spi_write(uint8_t data);
uint8_t spi_read(void);
void w25q64_write_enable(void);
void w25q64_write_byte(uint32_t addr, uint8_t data);
void w25q64_chip_erase(void);
void w25q64_sector_erase(uint32_t addr);