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

/************************************************************************/
/* Globals                                                              */
/************************************************************************/
volatile uint32_t g_ul_value = 0;
volatile uint32_t temp_value = 0;
volatile char temp_text[32];
volatile char texto[32];


/* Canal do sensor de temperatura */
#define AFEC_CHANNEL 0

////// AFEC
/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)


//ICONS

#include "ar.h"
#include "soneca.h"
#include "termometro.h"

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
#define AFEC_CHANNEL_RES_PIN 5

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;

/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_AFEC_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_AFEC_STACK_PRIORITY        (tskIDLE_PRIORITY)

typedef struct {
	uint x;
	uint y;
} touchData;


QueueHandle_t xQueueTouch;

/** Semaforo task led */
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

/*
static void AFEC_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	printf("but_callback \n");
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
	printf("semafaro tx \n");
}*/

static void AFEC_callback(void)
{
	temp_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL);
	
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

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
	font_draw_text(&digital52, "100%", 110, 250, 1);
	ili9488_draw_pixmap(30, 380, termometro.width, termometro.height, termometro.data);
	font_draw_text(&digital52, "15", 110, 380, 1);
	ili9488_draw_pixmap(210, 20, soneca.width, soneca.height, soneca.data);
}

void draw_temp(int t) {
	sprintf(temp_text, "%d", t);
	font_draw_text(&digital52, temp_text, 110, 380, 1);
}

void draw_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
	if(clicked) {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
		ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
		} else {
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
		ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
	}
	last_state = clicked;
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
	printf("%d", g_ul_value);
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
	configure_lcd();
	
	draw_screen();
	draw_temp(temp_value);
	//draw_button(0);
	// Escreve HH:MM no LCD
	font_draw_text(&digital52, "HH:MM", 0, 0, 1);
	
	touchData touch;
	
	while (true) {
		if (xQueueReceive( xQueueTouch, &(touch), ( TickType_t )  500 / portTICK_PERIOD_MS)) {
			update_screen(touch.x, touch.y);
			printf("x:%d y:%d\n", touch.x, touch.y);
		}
	}
}

void task_afec(void){

  while( true){
	  afec_start_software_conversion(AFEC0);
	  vTaskDelay(500);
	  draw_temp(convert_adc_to_temp(temp_value));
	  printf(" Temp AFEC: %d \r \n ", convert_adc_to_temp(temp_value));
	  
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
	ioport_init();
	
	/* inicializa e configura adc */
	config_ADC();
	
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

	/* Start the scheduler. */
	vTaskStartScheduler();

  while(1){

  }


  return 0;

}
