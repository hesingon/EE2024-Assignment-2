/*****************************************************************************
 *	EE2024 Assignment 2 AY1516 Sem 1
 *	Chen Si & Hang Zi Kai
 ******************************************************************************/

//////Lib_MCU//////
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_rtc.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_adc.h"



//////Protothread-inspired Scheduler//////
#include "2024_scheduler.h"

//////Peripheral Drivers//////
#include "2024_oled.h"
#include "stdio.h"
#include "string.h"
#include "2024_rgb.h"
#include "light.h"
#include "2024_pca9532.h"
#include "led7seg.h"
#include "acc.h"
#include "msg.h"
#include "monitor.h"


//////mission data//////
#define LIGHT_THRESHOLD 3000

//////global variables//////
uint8_t volatile hopeMode = 0; // 0 = EXPLORER, 1 = SURVIVAL
uint8_t volatile trigger_no = 0;
volatile int32_t sensor_time;			//sensor time
uint8_t temp_whole=0;				//temperature
uint8_t temp_dec=0;
SEM forceUpdate;
SEM forceUpdate1;
SEM LightIrq;
SEM trigger_add;
uint8_t num_led=16;
uint8_t led_dir=1;
int8_t xoff = 0;
int8_t yoff = 0;
int8_t zoff = 0;
volatile uint8_t elapsed_500ms = 0;
SEM check500;
SEM clear500;
uint8_t rotary_dir;
int adc_value;
RTC_TIME_Type current_time;


typedef struct {
	uint8_t valid;
	uint8_t elapsed;
}trigger;

trigger triggers[9];



static void init_trigger(){
	static uint8_t i;
	for(i=0;i<9;i++){
		triggers[i].valid=0;
		triggers[i].elapsed=0;
	}
}

static void init_ADC(){
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN0,ENABLE);
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_0,ENABLE);
}

static void init_GPIO() {
	GPIO_SetDir(1, 1 << 31, 0);		// Initialise SW PIO1_31
	GPIO_SetDir(2, 1, 1);			//init red led at P2.0
	GPIO_SetDir(0, (1 << 26), 1);	//init blue led at p1.26

	/*LPC_PINCON->PINSEL3 |= (2<<20);	//Setup P1.26 as PWM1.6
	LPC_PINCON->PINSEL4 |= 1;	//Setup P2.0 as PWM1.1*/

	LPC_PINCON->PINMODE3 |= (2<<20);	//neither pull-up or -down
	LPC_PINCON->PINMODE3 |= (2<<0);

	LPC_GPIOINT ->IO2IntEnF |= 1 << 5;
	LPC_GPIOINT ->IO2IntEnF |= 1 << 10;
	LPC_GPIOINT ->IO0IntEnF |= 1 << 24;
	NVIC_EnableIRQ(EINT3_IRQn);

    //Config P0.4 as CAP2.0 | Baseboard P0_1
	LPC_PINCON->PINSEL0 |= (3<<8);    //Setup P0.4 as CAP2.0

	//Config P0.23 as AD0.0 | Baseboard P0_11
	LPC_PINCON->PINSEL1 |= (1<<14);	  //Setup P0.23 as AD0.0
}

void EINT3_IRQHandler() {
	if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1) { //P2_5
		SendSem(LightIrq);
		LPC_GPIOINT ->IO2IntClr = (1 << 5);
	} else if ((LPC_GPIOINT ->IO2IntStatF >> 10) & 0x1) {
		SendSem(forceUpdate);
		if(hopeMode==1){
		SendSem(forceUpdate1);
		}
		LPC_GPIOINT ->IO2IntClr = (1 << 10);
	}
}

void LPC17xx_UART_PutChar (uint8_t ch)
{
	while (!(LPC_UART3->LSR & 0x20));
	LPC_UART3->THR = ch;
}

uint8_t LPC17xx_UART_GetChar (void)
{
	while (!(LPC_UART3->LSR & 0x01));
	return (LPC_UART3->RBR);
}

static void init_i2c_ssp_uart(void) {
	PINSEL_CFG_Type PinCfg;
	SSP_CFG_Type SSP_ConfigStruct;

	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);
	I2C_Init(LPC_I2C2, 800000);
	I2C_Cmd(LPC_I2C2, I2C_MASTER_MODE, ENABLE);

	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	SSP_ConfigStructInit(&SSP_ConfigStruct);
	SSP_ConfigStruct.ClockRate = 8000000;
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
	SSP_Cmd(LPC_SSP1, ENABLE);

	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 57600;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	//supply power & setup working par.s for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);

}

static void init_timer2(){
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM2, ENABLE);
	 LPC_TIM2->PR = (25-1);		//1 µs at 100MHz/4
	 LPC_TIM2->CCR =((1<<0)|(1<<2));   //capture rising edge with interrupt
	 LPC_TIM2->TCR = 1;                //start timer
	 NVIC_EnableIRQ(TIMER2_IRQn);
}

void TIMER2_IRQHandler(void){
	 LPC_TIM2->TC =0;						//reset timer
	 sensor_time = LPC_TIM2->CR0;			//store captured time
	 uint32_t reg_val;
	 reg_val = LPC_TIM2->IR;
	 if(reg_val & (1<<4))            		//CR0 interrupt
	 {
	  LPC_TIM2->IR = (1<<4);        	    //reset interrupt
	 }
}
//////task 7: uart messages//////
void task7(){
	xprintf( "%c[2J", 0x1b ); //clear screen
	xprintf("\n");
	xcolor(0,3,0);
	xprintf(string0);
	xprintf(string1);
	xprintf(string2);
	xprintf(string3);
	xprintf(string4);
	xprintf(string5);
	xprintf(string6);
	xprintf(string7);
	xprintf(string8);
	xprintf(string9);
	xprintf(string10);
	xprintf(string11);
	xprintf(string12);
	xprintf(string13);
	xprintf(string14);
	xprintf(string15);
	xprintf(string16);
	xprintf(string17);
	xprintf(string18);
	xprintf(string19);
	xprintf(string20);
	xprintf("System Core Clock = %d Hz \n",SystemCoreClock);
	RTC_GetFullTime(LPC_RTC, &current_time);
	xprintf("System Initialisation Success: %d-%d-%d - enter EXPLORER\n", current_time.DOM, current_time.MONTH, current_time.YEAR);
	xcolor(0,7,0);
	xprintf("\n");
}

//////task 6: initialisation sequence//////
unsigned char task6(){
_SS
	while(1){
		timers[0] = FOREVER;
		timers[1] = FOREVER;
		timers[2] = FOREVER;
		timers[3] = FOREVER;
		timers[4] = FOREVER;
		timers[5] = FOREVER;
		task7();		//uart welcome message
		oled_clearScreen(OLED_COLOR_BLACK);
		led7seg_setChar(':',0);
		WaitX(100);
		led7seg_setChar('1',0);
		WaitX(100);
		led7seg_setChar('2',0);
		WaitX(100);
		led7seg_setChar('3',0);
		WaitX(100);
		led7seg_setChar('4',0);
		WaitX(100);
		led7seg_setChar('5',0);
		WaitX(100);
		led7seg_setChar('6',0);
		WaitX(100);
		oled_putLogo();
		led7seg_setChar('A',0);
		WaitX(20);
		oled_clearScreen(OLED_COLOR_BLACK);
		WaitX(20);
		oled_putLogo();
		WaitX(60);
		led7seg_setChar('B',0);
		WaitX(100);
		led7seg_setChar('C',0);
		WaitX(100);
		led7seg_setChar('D',0);
		WaitX(100);
		led7seg_setChar('E',0);
		WaitX(100);
		led7seg_setChar('F',0);
		WaitX(100);
		led7seg_setChar('0',0);
		WaitX(100);
		oled_clearScreen(OLED_COLOR_BLACK);
		light_clearIrqStatus();
		timers[0] = 0;
		timers[1] = 0;
		timers[2] = 0;
		timers[3] = 0;
		timers[4] = 0;
		timers[5] = 0;

		WaitX(FOREVER);
	}
_EE
}

//////task 0: LED array //////
unsigned char task0(){
_SS
	while(1){
		if(hopeMode==1 && num_led>0){
			static uint16_t light;
			light = light_read();
			if(light < LIGHT_THRESHOLD)
			{
				setLeds(--num_led);
			}
			else
			{
				num_led=16;
				setLeds(num_led);
				WaitX(25);
			}
		}
		else if(hopeMode==1 && num_led == 0)
		{
			hopeMode = 0;
			RTC_GetFullTime(LPC_RTC, &current_time);
			xprintf("[%-2d:%-2d:%-2d] Lightning Has Subsided. Scheduled Telemetry Will Now Resume.\r\n", current_time.HOUR, current_time.MIN, current_time.SEC);
			SendSem(forceUpdate);
			SendSem(forceUpdate1);
			LED_R_OFF;
		}
		WaitX(25);
	}
_EE
}

//////task 1: LED indicator blink//////
unsigned char task1() {
_SS
	while (1) {
		if (hopeMode == 0){
			LED_B_TOG;	//explorer mode
		}
		else{
			LED_R_TOG;	//survival mode
		}
		WaitX(100);
	}
_EE
}

//////task 2: update sensor values, update display, send uart//////
unsigned char task2() {
_SS
while (1) {
    static uint16_t light_val;
    static char LightPrint[80];
    if(hopeMode == 0){
        WaitSemX(forceUpdate, 200);
    }
    else{
        WaitSem(forceUpdate);
    }
    if(hopeMode == 1){
        WaitSem(forceUpdate1);
    }
    light_val = light_read();
    xsprintf(LightPrint, "Light: %-5d lux", light_val);
    oled_putString(0, 8, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
    OLED_COLOR_BLACK);
    RTC_GetFullTime(LPC_RTC, &current_time);
    xsprintf(LightPrint, "%02d:", current_time.HOUR);
    xsprintf(LightPrint + 3, "%02d:", current_time.MIN);
    xsprintf(LightPrint + 6, "%02d", current_time.SEC);
    oled_putString(0, 0, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
    OLED_COLOR_BLACK);
    temp_whole = (sensor_time/160 - 273);
    temp_dec = (sensor_time/16 - 2731) % 10;
    xsprintf(LightPrint, "temp:  %d.%d   C", temp_whole , temp_dec);
    oled_putString(0, 16, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
    OLED_COLOR_BLACK);
    static int8_t x = 0;
    static int8_t y = 0;
    static int8_t z = 0;
    acc_read(&x, &y, &z);
    //x = x+xoff;
    //y = y+yoff;
    z = z+zoff;
    xsprintf(LightPrint, "x_acc: %-4d", x);
    oled_putString(0, 32, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
    OLED_COLOR_BLACK);
    xsprintf(LightPrint, "y_acc: %-4d", y);
    oled_putString(0, 40, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
    OLED_COLOR_BLACK);
    xsprintf(LightPrint, "z_acc: %-4d", z);
    oled_putString(0, 48, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
    OLED_COLOR_BLACK);
    xcolor(0,4,0);
    xprintf("[%-2d:%-2d:%-2d] ", current_time.HOUR, current_time.MIN, current_time.SEC);
    xcolor(0,7,0);
    xprintf("L%d_T%d.%d_AX%d_AY%d_AZ%d\n",light_val, temp_whole, temp_dec, x, y, z);

	}
_EE
}

//////task 3: handler for light sensor trigger//////
unsigned char task3() {
_SS
	while (1) {
		WaitSem(LightIrq);
		if(hopeMode == 1){
			num_led = 17;
		}
		elapsed_500ms=0;
		SendSem(check500);
		light_setMaxThreshold();
		light_setLoThreshold(LIGHT_THRESHOLD);
		light_clearIrqStatus();
		WaitSem(LightIrq);
		light_setMinThreshold();
		light_setHiThreshold(LIGHT_THRESHOLD);
		light_clearIrqStatus();
		if(!elapsed_500ms){
			SendSem(trigger_add);
			SendSem(clear500);
		}
	}
_EE
}

//////task8: check if trgger > 500ms //////
unsigned char task8(){
_SS
	while(1){
		WaitSem(check500);
		WaitSemX(clear500,50);	//wait either 500ms or the reset semaphore
		elapsed_500ms=1;
	}
_EE
}
//////task 4: remove expired triggers//////
unsigned char task4() {
_SS
	while (1) {

			static uint8_t i;
			for(i=0; i<9; i++){
				if(triggers[i].valid){
					triggers[i].elapsed++;

					if(triggers[i].elapsed == 30){
						triggers[i].valid=0;
						triggers[i].elapsed=0;
						trigger_no--;
						led7seg_setChar(trigger_no + '0', 0);
					}
					WaitX(0);
				}
			}
			WaitX(10);

	}
_EE
}


//////task 5: tracking triggers//////
unsigned char task5(){
_SS
	while(1){
		static uint8_t i;
		static uint8_t elapsed_max;
		static uint8_t elapsed_max_loc;
		WaitSem(trigger_add);
		if(trigger_no<9){				//fewer than 9 triggers - find an empty slot & add
			i=0;
			while(triggers[i].valid)
				i++;
			triggers[i].elapsed = 0;
			triggers[i].valid = 1;
			trigger_no++;
		}else{					// already have 9 triggers - reset the timer of the earliest
			elapsed_max=triggers[0].elapsed;
			elapsed_max_loc=0;
			for(i=1; i<9; i++){
				if(triggers[i].elapsed > elapsed_max){
					elapsed_max=triggers[i].elapsed;
					elapsed_max_loc=i;
				}
			}
			triggers[elapsed_max_loc].elapsed=0;
		}
		if(trigger_no == 3 && hopeMode == 0){
			num_led = 17;
			hopeMode = 1;
			static char LightPrint[20];
			RTC_GetFullTime(LPC_RTC, &current_time);
			xcolor(1,0,1);
			xprintf("[%-2d:%-2d:%-2d] Lightning Detected. Scheduled Telemetry is Temporarily Suspended.", current_time.HOUR, current_time.MIN, current_time.SEC);
			xcolor(0,7,0);
			xputc('\n');
			xsprintf(LightPrint, "x_acc: S      ");
			oled_putString(0, 32, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			xsprintf(LightPrint, "y_acc: S      ");
			oled_putString(0, 40, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			xsprintf(LightPrint, "z_acc: S      ");
			oled_putString(0, 48, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			xsprintf(LightPrint, "Light: S    ");
			oled_putString(0, 8, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			xsprintf(LightPrint, "temp:  S      ");
			oled_putString(0, 16, (uint8_t *) LightPrint, OLED_COLOR_WHITE,
					OLED_COLOR_BLACK);
			LED_B_OFF;
		}
		led7seg_setChar(trigger_no + '0', 0);
	}
_EE
}

//////task 9: ADC//////
unsigned char task9(){
_SS

	while(1){
		ADC_StartCmd(LPC_ADC,ADC_START_NOW);
		WaitUntil(ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_0,ADC_DATA_DONE));
		adc_value = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_0);
		setBright((uint8_t) (adc_value >> 4));
		WaitX(25);
	}
_EE
}


void SysTick_Handler(void) {
	UpdateTimers();
}

int main(void) {


	SysTick_Config(SystemCoreClock / 100 -1);
	init_GPIO();
	init_i2c_ssp_uart();
	init_ADC();
	//init_PWM();

	RTC_Init(LPC_RTC);
	RTC_Cmd(LPC_RTC, ENABLE);

	light_enable();
	light_setRange(LIGHT_RANGE_4000);
	light_setWidth(LIGHT_WIDTH_12BITS);
	light_setHiThreshold(LIGHT_THRESHOLD);
	light_setIrqInCycles(LIGHT_CYCLE_1);
	light_clearIrqStatus();

	led7seg_init();
	led7seg_setChar('0', 0);

	init_timer2();

	NVIC_SetPriorityGrouping(5);
	NVIC_SetPriority(TIMER2_IRQn, ((0x01 << 3) | 0x01));  // preemption = 1, sub-priority = 1
	NVIC_SetPriority(SysTick_IRQn, ((0x03 << 3) | 0x01));  // preemption = 2, sub-priority = 1
	NVIC_SetPriority(EINT3_IRQn, ((0x02 << 3) | 0x01));  // preemption = 2, sub-priority = 1

    xfunc_out = LPC17xx_UART_PutChar;
    xfunc_in  = LPC17xx_UART_GetChar;

	acc_init();
    acc_read(&xoff, &yoff, &zoff);
    xoff = 0-xoff;
    yoff = 0-yoff;
    zoff = 0-zoff;



	oled_init();
	oled_clearScreen(OLED_COLOR_BLACK);

	InitSem(forceUpdate);	//semaphore to force update of sensors
	InitSem(forceUpdate);	//to confirm update is forced in survival mode
	InitSem(LightIrq);	//semaphore to clear light trigger
	InitSem(trigger_add);
	InitSem(check500);
	InitSem(clear500);

	init_trigger();
	/*current_time.DOM = 9;
	current_time.DOW = 1;
	current_time.DOY = 313;
	current_time.HOUR = 3;
	current_time.MIN=2;
	current_time.MONTH = 11;
	current_time.SEC = 0;
	current_time.YEAR=2015;

	RTC_SetFullTime(LPC_RTC, &current_time);*/

	while (1) {
		RunTaskA(task8, 8);		//filter false lightning > 500ms
		RunTaskA(task6, 6);		//initialisation sequence
		RunTaskA(task3, 3);		//lightning trigger handler
		RunTaskA(task5, 5);		//monitoring number of lightning triggers
		RunTaskA(task2, 2);		//update sensor values at 2 sec interval
		RunTaskA(task4, 4);		//remove expired lightning triggers
		RunTaskA(task1, 1);		//blinking RGB LED indicator
		RunTaskA(task0, 0);		//LED array countdown for survival
		RunTaskA(task9, 9);		//ADC to control LED array brightness (extension)
	}

}

