#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "tfont.h"
#include "digital521.h"

//ICONS

#include "ar.h"
#include "soneca.h"
#include "termometro.h"
#include "fan.h"  //https://www.flaticon.com/free-icon/ac_95252

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
//volatile uint32_t temp_value = 0;
//volatile uint32_t adc_value = 0;
volatile uint32_t duty = 0;
volatile char temp_text[512];
volatile char fan_text[512];
volatile char duty_text[512];
volatile char texto[32];


/* Canal do sensor de temperatura */
#define AFEC_CHANNEL 0

// AFEC
/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)




/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        3

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

#define BUT_PIO           PIOD
#define BUT_PIO_ID        ID_PIOD
#define BUT_PIO_IDX       28u
#define BUT_IDX_MASK  (1u << BUT_PIO_IDX)

#define BUT2_PIO          PIOC
#define BUT2_PIO_ID        ID_PIOC
#define BUT2_PIO_IDX       31u
#define BUT2_IDX_MASK  (1u << BUT2_PIO_IDX)

/************************************************************************/
/* PWM                                                        */
/************************************************************************/

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

#define PIO_PWM_0 PIOA
#define ID_PIO_PWM_0 ID_PIOA
#define MASK_PIN_PWM_0 (1 << 0)

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;

/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_AFEC_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_AFEC_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct {
	uint x;
	uint y;
} touchData;


QueueHandle_t xQueueTouch;
QueueHandle_t xQueueDuty;
QueueHandle_t xQueueTemp;
//QueueHandle_t xQueueRealTemp;
QueueHandle_t xQueueAfec;

QueueHandle_t xQueuePot;

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSemaphore2;




/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
* \brief Called if stack overflow during execution
*/
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/**
* \brief This function is called by FreeRTOS idle task
*/
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
* \brief This function is called by FreeRTOS each tick
*/
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/


static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
* \brief Set maXTouch configuration
*
* This function writes a set of predefined, optimal maXTouch configuration data
* to the maXTouch Xplained Pro.
*
* \param device Pointer to mxt_device struct
*/
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
	MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	* the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	* value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}


/************************************************************************/
/* Callbacks: / Handler                                                 */
/************************************************************************/

static void AFEC_callback(void)
{
	int32_t temp_value;
	temp_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL);
	xQueueSendFromISR( xQueueAfec, &temp_value, 0);
	//temp_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL);
	
}

void but_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}

void but_callback2(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback2\n");
	xSemaphoreGiveFromISR(xSemaphore2, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void)
{
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but_callback2);

	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
}

void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = duty;
	g_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM0, &g_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}

void PWM1_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM1);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM1, PIN_PWM_LED1_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM1, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = duty;
	g_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM1, &g_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM1, channel);
}

static int32_t convert_adc_to_temp2(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_temp;

  /*
   * converte bits -> tens?o (Volts)
   */
	ul_vol = ADC_value * 100 / 4096;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  return(ul_vol);
}

static int32_t convert_adc_to_temp(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_temp;
	ul_temp = (100*ADC_value + 5900)/4026;
  return(ul_temp-3);
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	ili9488_draw_pixmap(20, 250, ar.width, ar.height, ar.data);
	ili9488_draw_pixmap(30, 380, termometro.width, termometro.height, termometro.data);
	ili9488_draw_pixmap(210, 20, soneca.width, soneca.height, soneca.data);
	ili9488_draw_pixmap(37, 130, fan.width, fan.height, fan.data);
}

void draw_temp(int temp) {
	sprintf(temp_text, "%2d C  ", temp);
	if(temp>70){
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
		}
	else if (temp > 50 && temp < 70){
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_YELLOW));
		}
	else if (temp > 30 && temp < 50){
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_ORANGE));
		}
	else if (temp > 10 && temp < 30){
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_CYAN));
	}
	else{
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLUE));
	}
	ili9488_draw_filled_rectangle(110,450,205,500);
	font_draw_text(&digital52, temp_text, 110, 380, 1);
	//ili9488_draw_filled_rectangle(14, ILI9488_LCD_HEIGHT - termometro.height+44, 19,ILI9488_LCD_HEIGHT - termometro.height - size +42 );
}

void draw_fan(int fan) {
	sprintf(fan_text, "%2d  ", fan);
	font_draw_text(&digital52, fan_text, 110, 130, 1);
}

void draw_duty(int duty) {
	sprintf(duty_text, "%2d", duty);
	font_draw_text(&digital52, duty_text, 110, 250, 1);
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void update_screen(uint32_t tx, uint32_t ty) {
	if(tx >= BUTTON_X-BUTTON_W/2 && tx <= BUTTON_X + BUTTON_W/2) {
		if(ty >= BUTTON_Y-BUTTON_H/2 && ty <= BUTTON_Y) {
			} else if(ty > BUTTON_Y && ty < BUTTON_Y + BUTTON_H/2) {
		}
	}
}

void mxt_handler(struct mxt_device *device, uint *x, uint *y)
{
	/* USART tx buffer initialized to 0 */
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;
	
	/* first touch only */
	uint first = 0;

	/* Collect touch events and put the data in a string,
	* maximum 2 events at the time */
	do {

		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		/************************************************************************/
		/* Envia dados via fila RTOS                                            */
		/************************************************************************/
		if(first == 0 ){
			*x = convert_axis_system_x(touch_event.y);
			*y = convert_axis_system_y(touch_event.x);
			first = 1;
		}
		
		i++;

		/* Check if there is still messages in the queue and
		* if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));
}


static void config_ADC(void){
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_callback, 5);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa convers?o */
	afec_channel_enable(AFEC0, AFEC_CHANNEL);
}

/************************************************************************/
/* tasks                                                                */
/************************************************************************/

void task_mxt(void){
  
  	struct mxt_device device; /* Device data container */
  	mxt_init(&device);       	/* Initialize the mXT touch device */
    touchData touch;          /* touch queue data type*/
    
  	while (true) {  
		  /* Check for any pending messages and run message handler if any
		   * message is found in the queue */
		  if (mxt_is_message_pending(&device)) {
		  	mxt_handler(&device, &touch.x, &touch.y);
        xQueueSend( xQueueTouch, &touch, 0);           /* send mesage to queue */
      }
     vTaskDelay(100);
	}
}

void task_lcd(void){
	xQueueTouch = xQueueCreate( 10, sizeof( touchData ) );
	xQueueDuty = xQueueCreate( 10, sizeof( int32_t ) );
	xQueueTemp = xQueueCreate( 10, sizeof( int32_t ) );
	
	configure_lcd();
	draw_screen();
	//Escreve HH:MM no LCD
	font_draw_text(&digital52, "HH:MM", 0, 0, 1);
	
	
	touchData touch;
	int32_t temp = 25;
	int32_t real_temp = 25;
	int32_t duty = 0;
	
	while (true) {
		//
		if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
			update_screen(touch.x, touch.y);
			
			printf("x:%d y:%d\n", touch.x, touch.y);
		}
		if (xQueueReceive(xQueueDuty, &(duty), ( TickType_t )  10 / portTICK_PERIOD_MS)) {
			printf("\n potencia: %d", duty);
			draw_duty(duty);
			vTaskDelay(300 / portTICK_PERIOD_MS);
		}
		if (xQueueReceive(xQueueTemp, &(temp), ( TickType_t )  10 / portTICK_PERIOD_MS)) {
			draw_temp(temp);
			//pot = (temp < real_temp) ? (((real_temp - temp) * 100 ) / (100 - temp)) : 0;
			//pot = 100 * (real_temp - temp) / (100 - temp);
			//xQueueSend(xQueuePot, &pot, 0);
			//draw_duty(pot);
			
			printf("\ntemperatura desejada: %d", temp);
		}

	}
}

static void task_pwm(void){
	const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
	
	xQueuePot = xQueueCreate( 10, sizeof( int32_t ) );
	int32_t pot;
	
	pmc_enable_periph_clk(ID_PIO_PWM_0);
	pio_set_peripheral(PIO_PWM_0, PIO_PERIPH_A, MASK_PIN_PWM_0 );
	PWM0_init(0, duty);

	while (1) {
		if (xQueueReceive( xQueuePot, &(pot), ( TickType_t ) 10 / portTICK_PERIOD_MS)) {
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led,(100 - pot));
			printf("duty: %d", pot);
			vTaskDelay(300 / portTICK_PERIOD_MS);
			xQueueSend(xQueueDuty, &pot, 0);
			
		}
	}
}

void task_afec(void){
	xQueueAfec = xQueueCreate( 10, sizeof( int32_t ) );

	config_ADC();
	afec_start_software_conversion(AFEC0);
	
	int32_t adc_value;
	int32_t temp_value;

	while (true) {
		if (xQueueReceive( xQueueAfec, &(adc_value), ( TickType_t )  4000 / portTICK_PERIOD_MS)) {
			temp_value = convert_adc_to_temp(adc_value);
			//printf("Temp : %d \r\n", temp_value);
			afec_start_software_conversion(AFEC0);
			xQueueSend( xQueueTemp, &temp_value, 0);
		}
	}
/*
  while(true){
	  afec_start_software_conversion(AFEC0);
	  vTaskDelay(500);
	  draw_temp(convert_adc_to_temp(temp_value));
	  printf(" Temp AFEC: %d \r \n ", convert_adc_to_temp(temp_value));
  }
  */


}
	
static void task_led(void *pvParameters)
{
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphore2 = xSemaphoreCreateBinary();
	int32_t duty = 0;
	//printf("%d", duty);
	
	//sprintf(texto, "%d", duty);
    io_init();

	if (xSemaphore == NULL)
		printf("falha em criar o semaforo \n");
	if (xSemaphore2 == NULL)
	printf("falha em criar o semaforo2 \n");

	while (1) {
		if( xSemaphoreTake(xSemaphore, ( TickType_t ) 500) == pdTRUE && duty < 100){
			duty = duty + 10;
			printf("%d", duty);
		}
		if( xSemaphoreTake(xSemaphore2, ( TickType_t ) 500) == pdTRUE && duty > 0 ){
			duty = duty - 10;
			printf("%d", duty);
			
		}
		xQueueSend( xQueuePot, &duty, 0);
		
	}
}
	
	/*
	xSemaphore = xSemaphoreCreateBinary();
	for (;;) {
		if( xSemaphoreTake(xSemaphore, ( TickType_t ) 10) == pdTRUE ){
			
			//100ms
			//const TickType_t xDelay = 100/ portTICK_PERIOD_MS;
			g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL);
			printf("%d", g_ul_value);
			char b[512];
			sprintf(b, g_ul_value);
			font_draw_text(&digital52, b, 110, 380, 1);
		}
	}/*
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	
	
	
	/* Create task to handler touch */
	if (xTaskCreate(task_mxt, "mxt", TASK_MXT_STACK_SIZE, NULL, TASK_MXT_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to handler LCD */
	if (xTaskCreate(task_lcd, "lcd", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to AFEC converter */
	if (xTaskCreate(task_afec, "afec", TASK_AFEC_STACK_SIZE, NULL, TASK_AFEC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to PWM converter */
	if (xTaskCreate(task_pwm, "pwm", TASK_AFEC_STACK_SIZE, NULL, TASK_AFEC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to PWM converter */
	if (xTaskCreate(task_led, "led", TASK_AFEC_STACK_SIZE, NULL, TASK_AFEC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();
	

  while(1){

  }


  return 0;

}
