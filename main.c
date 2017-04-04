/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include <stdio.h>
#include <stdlib.h>

#include "led7seg.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "light.h"
#include "temp.h"

typedef enum {
    Monitor,
    Stable,
} Mode;

Mode currMode=Stable;

static const int LIGHT_LOW_WARNING = 50;
static const float TEMP_HIGH_WARNING = 25.0;
static const int MOVEMENT_THRESHOLD = 4;

Bool alert;

int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;

int8_t x = 0;
int8_t y = 0;
int8_t z = 0;

int8_t xPrev = 0;
int8_t yPrev = 0;
int8_t zPrev = 0;

int lightReading;
float tempReading;

volatile uint32_t msTicks; // counter for 1ms SysTicks

// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
    msTicks++;
}

static void init_ssp(void)
{
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    /*
     * Initialize SPI pin connect
     * P0.7 - SCK;
     * P0.8 - MISO
     * P0.9 - MOSI
     * P2.2 - SSEL - used as GPIO
     */
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

    // Initialize SSP peripheral with parameter given in structure above
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

    // Enable SSP peripheral
    SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
    PINSEL_CFG_Type PinCfg;

    /* Initialize I2C2 pin connect */
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    // Initialize I2C peripheral
    I2C_Init(LPC_I2C2, 100000);

    /* Enable I2C1 operation */
    I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
    // Initialize button
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg); // SW4
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 4;
    PINSEL_ConfigPin(&PinCfg); // SW3
    PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg); // Temp sensor
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PINSEL_ConfigPin(&PinCfg); // RGB B
    PinCfg.Pinnum = 3;
    PINSEL_ConfigPin(&PinCfg); // acc interrupt (may or may not use)
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 0;
    PINSEL_ConfigPin(&PinCfg); // RGB R
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg); // RGB G
    PinCfg.Pinnum = 5;
    PINSEL_ConfigPin(&PinCfg); // Temp sensor interrupt pin

    GPIO_SetDir( 2, (1<<0), 1 );
    GPIO_SetDir( 0, (1<<26), 1 );
    GPIO_SetDir( 2, (1<<1), 1 );

    // ---- Speaker ------>
    /*
    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn
    */
    // <---- Speaker ------

    //PINSEL_CFG_Type PinCfg;
    //PinCfg.Funcnum = 0;
    //PinCfg.OpenDrain = 0;
    //PinCfg.Pinmode = 0;
    //PinCfg.Portnum = 2;
    //PinCfg.Pinnum = 10;
    //PINSEL_ConfigPin(&PinCfg);
}

void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

static void init_uartWired(void){

    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;

    pinsel_uart3();

    UART_Init(LPC_UART3, &uartCfg);

    UART_TxCmd(LPC_UART3, ENABLE);
}

static uint32_t getTicks()
{
    return msTicks;
}

uint8_t numToChar(int num) {
    if (num < 10)
        return (uint8_t) (num + 48);
    if (num > 9)
        return (uint8_t) (num + 55);
    else
        return (uint8_t) num; //stub
}

void acc_setup() {
    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0 - x;
    yoff = 0 - y;
    zoff = 64 - z;
    xPrev=x;
    yPrev=y;
    zPrev=z;
    acc_read(&x,&y,&z);
    x = x+xoff;
    y = y+yoff;
    z = z+zoff;
    xPrev=x;
    yPrev=y;
    zPrev=z;

}

static void init_all() {
    init_i2c();
    init_ssp();
    init_GPIO();

    init_uartWired();

    rgb_init();
    acc_init();
    oled_init();
    led7seg_init();
    temp_init(&getTicks);

    light_enable();
    light_setRange(LIGHT_RANGE_4000);
    light_setLoThreshold(LIGHT_LOW_WARNING);
    light_setIrqInCycles(LIGHT_CYCLE_1);
    light_clearIrqStatus();

    LPC_GPIOINT->IO2IntEnF |= 1<<5; //GPIO interrupt P2.5 for light sensor
    NVIC_EnableIRQ(EINT3_IRQn); // enable EINT3 interrupt


    oled_clearScreen(OLED_COLOR_WHITE);
}

void EINT3_IRQHandler(void)
{
//  int i;
    // Determine whether GPIO Interrupt P2.5 has occurred
    if ((LPC_GPIOINT->IO2IntStatF>>5)& 0x1)
    {
        printf("LOW LIGHT WARNING\n");
        lightReading=light_read();
        light_clearIrqStatus();
        LPC_GPIOINT->IO2IntClr |= 1<<5;
    }
}

int btn1Press() { //SW4
    return (GPIO_ReadValue(1) >> 31) & 0x01;
}

int btn2Press() { //SW3
    return (GPIO_ReadValue(0) >> 4) & 0x01;
}

/* unable to make this work...yet
void blink(int color) {
    Bool toggle=TRUE;
    uint32_t currTime = getTicks();

    if(getTicks()-currTime>333)
    {
        if(toggle)
        {
            rgb_setLeds (4+color);
        }
        else{
            rgb_setLeds(4);
        }
        toggle=!toggle;
        currTime = getTicks();
    }
}
*/

void acc_read_improved(int8_t *x, int8_t *y, int8_t *z) {
    xPrev=*x;
    yPrev=*y;
    zPrev=*z;
    acc_read(x,y,z);
    *x = *x+xoff;
    *y = *y+yoff;
    *z = *z+zoff;
}

int isMoving() {
    return(abs(xPrev-x)/MOVEMENT_THRESHOLD ||
            abs(yPrev-y)/MOVEMENT_THRESHOLD ||
            abs(zPrev-z)/MOVEMENT_THRESHOLD);
    //return 1; // stub
}

void stableMode() {
    while(currMode==Stable)
    {
        oled_clearScreen(OLED_COLOR_BLACK); // clear oled
        led7seg_setChar(' ', FALSE); // 7seg switch off
        rgb_setLeds(4); // rgb switch off
        alert=FALSE; // turn off warnings

        if (!btn1Press()) {
            currMode=Monitor;
            printf("GOING MONITOR\n");
            break;
        }
    }
}

void monitorMode() {

    uint8_t ch = 16;

    uint32_t currTime = getTicks();
    uint32_t blinkTime = getTicks();
    //uint32_t sensorTime = getTicks();

    int a=0,b=0;

    char temp_sensor_value[40];
    char light_sensor_val[40];
    char acc_sensor_value[3][40];
    Bool toggle=FALSE;
    Bool displayed=FALSE;

    acc_setup();

    oled_clearScreen(OLED_COLOR_WHITE);

    while (currMode==Monitor)
    {
        if(tempReading> TEMP_HIGH_WARNING) {
            alert=TRUE;
            a=1;
        }
        if((lightReading< LIGHT_LOW_WARNING) && isMoving()) {
            alert=TRUE;
            b=2;
        }

        if(alert){
            if((getTicks() - blinkTime)>166)
            {
                if(toggle){
                    rgb_setLeds(4+a+b);
                }
                else{
                    rgb_setLeds(4);
                }
                toggle = !toggle;
                blinkTime = getTicks();
            }

        }


        oled_putString(1,1,(uint8_t *)"MONITOR",OLED_COLOR_BLACK,OLED_COLOR_WHITE);

        if (!btn1Press()) {
            currMode=Stable;
            printf("GOING STABLE\n");
            break;
        }

        //Display info on oLED, on '5' 'A' 'F'
        if(((ch==6)||(ch==11)||(ch==16))&&(!displayed)) {
            //temp sensor
            tempReading=temp_read()/10.0;
            //printf("temp: %.1f\n", tempReading);
            sprintf(temp_sensor_value,"Temp: %.1f",tempReading);

            //light sensor
            lightReading = light_read();
            //printf("light: %d\n", lightReading);
            sprintf(light_sensor_val, "Light: %d  " , lightReading);

            //accelerometer
            acc_read_improved(&x, &y, &z);

            //printf("acc: x:%d y:%d z:%d \n", x, y, z);
            sprintf(acc_sensor_value[0],"Acc:   x:%d  ", x);
            sprintf(acc_sensor_value[1],"     y:%d  ", y);
            sprintf(acc_sensor_value[2],"     z:%d  ", z);

            oled_putString(1,9,(uint8_t *)temp_sensor_value,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
            oled_putString(1,17,(uint8_t *)light_sensor_val,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
            oled_putString(1,25,(uint8_t *)acc_sensor_value[0],OLED_COLOR_BLACK,OLED_COLOR_WHITE);
            oled_putString(1,33,(uint8_t *)acc_sensor_value[1],OLED_COLOR_BLACK,OLED_COLOR_WHITE);
            oled_putString(1,41,(uint8_t *)acc_sensor_value[2],OLED_COLOR_BLACK,OLED_COLOR_WHITE);

            displayed=TRUE;
        }
        //conditions
        if (ch==16) ch=0; //char rollover, 7seg

        if((getTicks()-currTime)>1000)
        {
            //resets display token
            displayed=FALSE;
            //7seg
            led7seg_setChar(numToChar(ch++), FALSE);
            //reconfig currTime
            currTime=getTicks();
        }

    }
}

int main (void) {

//sysTick
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);  // Capture error
    }

    init_all();

    while (1)
    {
        if(currMode==Monitor) monitorMode();
        if(currMode==Stable) stableMode();
    }


}

void check_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while(1);
}

