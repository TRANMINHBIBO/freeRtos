#include "atc_e25.h"
#include "gd32f30x.h"
#include "gd32f30x_gpio.h"
#include "gd32f30x_rcu.h"
#include "systick.h"
#include "gd32f30x_exti.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "freeRtos.h"
#include "task.h"
#include "gd32f30x_i2c.h"
#include "gd32f30x_spi.h"
uint8_t received_data[100];
uint8_t rcv_index=0;
uint8_t received_data2[100];
//init led RGB on board
void atc25_led_init(void)
{
	rcu_periph_clock_enable(LED_BLUE_CLK);
	gpio_init(LED_BLUE_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,LED_BLUE_PIN);
	gpio_bit_set(LED_BLUE_PORT,LED_BLUE_PIN); //turn off led blue
	
	rcu_periph_clock_enable(LED_GREEN_CLK);
	gpio_init(LED_GREEN_PORT,GPIO_MODE_OUT_PP,GPIO_OSPEED_50MHZ,LED_GREEN_PIN|LED_RED_PIN);
	gpio_bit_set(LED_GREEN_PORT,LED_GREEN_PIN|LED_RED_PIN); //turn off led green & red
}

void atc25_btn1_init(keymode_typedef_enum btn_mode)
{
	if(btn_mode!=KEY_MODE_EXTI)
		{
			rcu_periph_clock_enable(BTN1_GPIO_CLK);
			gpio_init(BTN1_GPIO_PORT,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,BTN1_PIN);
		}
	else
		{
			rcu_periph_clock_enable(BTN1_GPIO_CLK);
			rcu_periph_clock_enable(RCU_AF);
			gpio_init(BTN1_GPIO_PORT,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,BTN1_PIN);
			
			nvic_irq_enable(BTN1_EXTI_IRQn,1U,0U);
			/* connect key EXTI line to key GPIO pin */
      gpio_exti_source_select(BTN1_EXTI_PORT_SOURCE,BTN1_EXTI_PIN_SOURCE);
			/* configure key EXTI line */
			exti_init(BTN1_EXTI_LINE,EXTI_INTERRUPT,EXTI_TRIG_BOTH);
			//exti_interrupt_enable(BTN1_EXTI_LINE);
			exti_interrupt_flag_clear(BTN1_EXTI_LINE);
		}
}

void atc25_btn2_init(keymode_typedef_enum btn_mode)
{
	if(btn_mode!=KEY_MODE_EXTI)
		{
			rcu_periph_clock_enable(BTN2_GPIO_CLK);
			gpio_init(BTN2_GPIO_PORT,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,BTN2_PIN);
		}
	else
		{
			rcu_periph_clock_enable(BTN2_GPIO_CLK);
			rcu_periph_clock_enable(RCU_AF);
			gpio_init(BTN2_GPIO_PORT,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,BTN2_PIN);
			
			nvic_irq_enable(BTN2_EXTI_IRQn,2U,0U);
			/* connect key EXTI line to key GPIO pin */
      gpio_exti_source_select(BTN2_EXTI_PORT_SOURCE,BTN2_EXTI_PIN_SOURCE);
			/* configure key EXTI line */
			exti_init(BTN2_EXTI_LINE,EXTI_INTERRUPT,EXTI_TRIG_BOTH);
			//exti_interrupt_enable(BTN2_EXTI_LINE);
			exti_interrupt_flag_clear(BTN2_EXTI_LINE);
		}
}

//turn on specific LED (RED/GREEN/BLUE) or All LED on the board
void atc25_led_on(ledlist_typedef_enum led_num)
{
	if(led_num==RED)
		gpio_bit_reset(LED_RED_PORT,LED_RED_PIN);
	else if(led_num==GREEN)
		gpio_bit_reset(LED_GREEN_PORT,LED_GREEN_PIN);
	else if(led_num==BLUE)
		gpio_bit_reset(LED_BLUE_PORT,LED_BLUE_PIN);
	else
	{
		gpio_bit_reset(LED_GREEN_PORT,LED_GREEN_PIN|LED_RED_PIN);
		gpio_bit_reset(LED_BLUE_PORT,LED_BLUE_PIN);
	}
}

//turn off LED
void atc25_led_off(ledlist_typedef_enum led_num)
{
	if(led_num==RED)
		gpio_bit_set(LED_RED_PORT,LED_RED_PIN);
	else if(led_num==GREEN)
		gpio_bit_set(LED_GREEN_PORT,LED_GREEN_PIN);
	else if(led_num==BLUE)
		gpio_bit_set(LED_BLUE_PORT,LED_BLUE_PIN);
	else
	{
		gpio_bit_set(LED_GREEN_PORT,LED_GREEN_PIN|LED_RED_PIN);
		gpio_bit_set(LED_BLUE_PORT,LED_BLUE_PIN);
	}
}
//LED running program
void atc25_led_running(uint32_t time_delay_ms)
{
	atc25_led_on(RED);
	vTaskDelay(pdMS_TO_TICKS(time_delay_ms));
	atc25_led_off(ALL);
	atc25_led_on(GREEN);
	vTaskDelay(pdMS_TO_TICKS(time_delay_ms));
	atc25_led_off(ALL);
	atc25_led_on(BLUE);
	vTaskDelay(pdMS_TO_TICKS(time_delay_ms));
	atc25_led_off(ALL);
	atc25_led_on(RED);
	vTaskDelay(pdMS_TO_TICKS(time_delay_ms));
	atc25_led_off(ALL);
	atc25_led_on(GREEN);
	vTaskDelay(pdMS_TO_TICKS(time_delay_ms));
	atc25_led_off(ALL);
	atc25_led_on(BLUE);
	vTaskDelay(pdMS_TO_TICKS(time_delay_ms));
	atc25_led_off(ALL);
}

void uart3_init(void){
	rcu_periph_clock_enable(RCU_UART3);
	gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
	gpio_init(GPIOC,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
	usart_enable(UART3);
	usart_word_length_set(UART3,USART_WL_8BIT);
	usart_stop_bit_set(UART3,USART_STB_1BIT);
	usart_baudrate_set(UART3,115200U);
	usart_receive_config(UART3,USART_RECEIVE_ENABLE);
	usart_transmit_config(UART3,USART_TRANSMIT_ENABLE);
	usart_parity_config(UART3,USART_PM_NONE);
	usart_hardware_flow_cts_config(UART3,USART_CTS_DISABLE);
	usart_hardware_flow_rts_config(UART3,USART_RTS_DISABLE);
}

void usart_string_transmit(uint32_t usart_periph, uint8_t data[])
{
	uint8_t i;
	for(i=0;i<strlen(data); i++)
	{
		usart_data_transmit(usart_periph,data[i]);
		while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
	}
}

void atc25_led_flash(ledlist_typedef_enum led_num, uint32_t time_ms)
{
	atc25_led_off(ALL);
	atc25_led_on(led_num);
	vTaskDelay(pdMS_TO_TICKS(time_ms));
	atc25_led_off(led_num);
	vTaskDelay(pdMS_TO_TICKS(time_ms));
	atc25_led_on(led_num);
	vTaskDelay(pdMS_TO_TICKS(time_ms));
	atc25_led_off(led_num);
	vTaskDelay(pdMS_TO_TICKS(time_ms));
}
bool check = TRUE;
void UART3_IRQHandler(void){
	if(usart_interrupt_flag_get(UART3, USART_INT_FLAG_RBNE) == SET){		
		received_data[rcv_index++] = usart_data_receive(UART3);	
				
		if(rcv_index == 100) rcv_index = 0;
		if(received_data[rcv_index - 1] == 13){
			uint8_t i; uint8_t j;
			received_data[rcv_index] = 10;
			for(i = 0; i <= rcv_index; i++) received_data2[i] = received_data[i];
			for(j = 0; j <= rcv_index; j++) received_data[i] = 0;
			rcv_index = 0;
		}
		check = TRUE;
		usart_interrupt_flag_clear(UART3, USART_INT_FLAG_RBNE);
	}
	else{
		check = FALSE;
	}
}
bool atc25_check_string(char* str){
	if(check && (strstr((char*)received_data2, str) != NULL)){
		uint8_t i;
		for(i = 0; i <= 99; i++) received_data2[i] = 0;	
		return TRUE;
	}
	else{
		return FALSE;
	}
}
void neoway_init(void){
	while(!atc25_check_string("OK")){
		usart_string_transmit(UART3,(uint8_t*)"AT\r\n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(RED, 200);
	while(!atc25_check_string("READY")){
		usart_string_transmit(UART3, (uint8_t*)"AT+CPIN?\r\n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(GREEN, 200);
	while(atc25_check_string("+CSQ: 99,99")){
		usart_string_transmit(UART3, (uint8_t*)"AT+CSQ\r\n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(BLUE, 200);
	while(!atc25_check_string("0,1") && !atc25_check_string("0,5")){
		usart_string_transmit(UART3, (uint8_t*)"AT+CEREG?\r\n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(RED, 200);
	while(!atc25_check_string("OK")){
		usart_string_transmit(UART3, (uint8_t*)"AT+XIIC=1\r\n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(GREEN, 200);
	while(!atc25_check_string("OK")){
		usart_string_transmit(UART3, (uint8_t*)"AT+XIIC?\r\n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(BLUE, 200);
	while(!atc25_check_string("OK")){
		neoway_set_param("mqttx_1819203438", "", "");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(RED, 200);
	while(!atc25_check_string("OK")){
		neoway_connect("91.121.93.94:1883", 0, 60);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	atc25_led_flash(GREEN, 200);	
}

void neoway_set_param(char* clientID, char* username, char* password){
	char command[100];
	sprintf(command, "AT+MQTTCONNPARAM=\"%s\",\"%s\",\"%s\"\r\n", clientID, username, password);
	usart_string_transmit(UART3, (uint8_t*)command);
}
void neoway_connect(char* host, uint8_t clean, uint8_t keep_alive){
	char command[100];
	sprintf(command, "AT+MQTTCONN=\"%s\",%d,%d\r\n", host, clean, keep_alive);
	usart_string_transmit(UART3, (uint8_t*)command);
}
void neoway_subscribe(char* topic_name, uint8_t qos){
	while(!atc25_check_string("OK")){
		char command[100];
		sprintf(command, "AT+MQTTSUB=\"%s\", %d\r\n", topic_name, qos);
		usart_string_transmit(UART3, (uint8_t*)command);
		vTaskDelay(pdMS_TO_TICKS(20));
	}
	neoway_pub(0, 1, "new_topic_lixy", (char*)received_data);
}

void neoway_pub(uint8_t retained, uint8_t qos, char* topicname, char* message){
	char command[100];
	sprintf(command, "AT+MQTTPUB=%d,%d,\"%s\",\"%s\"\r\n", retained, qos, topicname, message);
	usart_string_transmit(UART3, (uint8_t*)command);
}



void i2c_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(I2C1);
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10|GPIO_PIN_11);
	i2c_clock_config(I2C1, 100000U, I2C_DTCY_2);
	i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0X50);
	i2c_enable(I2C1);
	i2c_ack_config(I2C1, I2C_ACK_ENABLE);
	i2c_stretch_scl_low_config(I2C1, I2C_SCLSTRETCH_ENABLE);
	i2c_slave_response_to_gcall_config(I2C1, I2C_GCEN_ENABLE);
}
void eeprom_write(uint32_t i2c_periph, uint16_t addr, uint8_t data)
{
	i2c_start_on_bus(i2c_periph);
	while(i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)==RESET);
	i2c_data_transmit(i2c_periph, (AT24C256_ADDR | ((addr >> 7) & 0x0E)));
	while(i2c_flag_get(i2c_periph, I2C_FLAG_TBE)==RESET);
	i2c_data_transmit(i2c_periph, (uint8_t)(addr & 0xFF));
	while(i2c_flag_get(i2c_periph, I2C_FLAG_TBE)==RESET);
	i2c_data_transmit(i2c_periph, data);
	while(i2c_flag_get(i2c_periph, I2C_FLAG_TBE)==RESET);
	i2c_stop_on_bus(i2c_periph);
	while(i2c_flag_get(i2c_periph, I2C_FLAG_STPDET)==RESET);
}

uint8_t eeprom_read(uint32_t i2c_periph, uint16_t addr)
{
	uint8_t data;
	i2c_start_on_bus(i2c_periph);
	while(i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)==RESET);
	i2c_data_transmit(i2c_periph, (AT24C256_ADDR | ((addr >> 7) & 0x0E)));
	while(i2c_flag_get(i2c_periph, I2C_FLAG_TBE)==RESET);
	i2c_data_transmit(i2c_periph, (uint8_t)(addr & 0xFF));
	while(i2c_flag_get(i2c_periph, I2C_FLAG_TBE)==RESET);
	i2c_start_on_bus(i2c_periph);
  while(i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)==RESET);
	i2c_data_transmit(i2c_periph, (AT24C256_ADDR | 0x01));
  while(i2c_flag_get(i2c_periph, I2C_FLAG_TBE) == RESET);
	while(i2c_flag_get(i2c_periph, I2C_FLAG_RBNE) == RESET);
  data = i2c_data_receive(i2c_periph);
	i2c_stop_on_bus(i2c_periph);
  while (i2c_flag_get(i2c_periph, I2C_FLAG_STPDET) == RESET); 

  return data;
}

void spi_config(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	rcu_periph_clock_enable(RCU_SPI0);
	spi_parameter_struct spi_init_struct;
	//spi_struct_para_init(&spi_init_struct);
	spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX; 
	spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT; 
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss = SPI_NSS_SOFT;  
	spi_init_struct.prescale = SPI_PSC_32;
	spi_init_struct.endian = SPI_ENDIAN_MSB; 
	spi_init(SPI0, &spi_init_struct);
	spi_enable(SPI0);
}

void spi_write(uint8_t data) {
    spi_i2s_data_transmit(SPI0, data);
    while (spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) == RESET);
    while (spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) == RESET);
    uint8_t dummy = spi_i2s_data_receive(SPI0); 
}

uint8_t spi_read(void) {
    spi_i2s_data_transmit(SPI0, 0xFF); 
    while (spi_i2s_flag_get(SPI0, SPI_FLAG_TBE) == RESET);
    while (spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE) == RESET);
    return spi_i2s_data_receive(SPI0); 
}

void w25q64_write_enable(void) {
    gpio_bit_reset(GPIOA, GPIO_PIN_4); 
    spi_write(0x06); 
    gpio_bit_set(GPIOA, GPIO_PIN_4); 
}

void w25q64_write_byte(uint32_t addr, uint8_t data) {
    w25q64_write_enable(); 
    gpio_bit_reset(GPIOA, GPIO_PIN_4); 
    spi_write(0x02); 
    spi_write((addr >> 16) & 0xFF); 
    spi_write((addr >> 8) & 0xFF); 
    spi_write(addr & 0xFF); 
    spi_write(data); 
    gpio_bit_set(GPIOA, GPIO_PIN_4); 
    vTaskDelay(pdMS_TO_TICKS(5));
}
void w25q64_chip_erase(void) {
    w25q64_write_enable(); 
    gpio_bit_reset(GPIOA, GPIO_PIN_4); // CS = 0
    spi_write(0xC7); 
    gpio_bit_set(GPIOA, GPIO_PIN_4); // CS = 1
    vTaskDelay(pdMS_TO_TICKS(5000));
}
void w25q64_sector_erase(uint32_t addr) {
    w25q64_write_enable(); 
    gpio_bit_reset(GPIOA, GPIO_PIN_4); // CS = 0
    spi_write(0x20); 
    spi_write((addr >> 16) & 0xFF); 
    spi_write((addr >> 8) & 0xFF); 
    spi_write(addr & 0xFF); 
    gpio_bit_set(GPIOA, GPIO_PIN_4); // CS = 1
    vTaskDelay(pdMS_TO_TICKS(3000));
}