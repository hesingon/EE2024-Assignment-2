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

#include <stdio.h>

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

Mode currMode=Monitor;

static const int LIGHT_LOW_WARNING = 50;
static const float TEMP_HIGH_WARNING = 45.0;

Bool LowLightFlag;
Bool HighTempFlag;
Bool alert;

int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;

int8_t x = 0;
int8_t y = 0;
int8_t z = 0;

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
}

static void init_all() {
    init_i2c();
    init_ssp();
    init_GPIO();

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
        LowLightFlag=TRUE;
        alert=TRUE;
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

int isMoving() {
    return 1; // stub
}

void stableMode() {
    while(currMode==Stable)
    {
        oled_clearScreen(OLED_COLOR_BLACK); // clear oled
        led7seg_setChar(' ', FALSE); // 7seg switch off
        rgb_setLeds(4); // rgb switch off

        if (!btn1Press()) {
            currMode=Monitor;
            printf("GOING MONITOR\n");
            break;
        }
    }
}

void monitorMode() {

    uint8_t ch = 0;

    uint32_t currTime = getTicks();

    int a=0,b=0;
    int lightReading;
    float tempReading;

    char temp_sensor_value[40];
    char light_sensor_val[40];
    char acc_sensor_value[3][40];
    Bool toggle=TRUE;

    acc_setup();

    LowLightFlag=FALSE;
    HighTempFlag=FALSE;

    oled_clearScreen(OLED_COLOR_WHITE);

    while (currMode==Monitor)
    {
        if(tempReading> TEMP_HIGH_WARNING) {
            alert=TRUE;
            a=1;
        }
        if(LowLightFlag && isMoving()) {
            b=2;
        }
        if(alert){

                if((getTicks()-currTime)>333)
                {
                    if(toggle){
                        rgb_setLeds(4+a+b);
                    }
                    else{
                        rgb_setLeds(4);
                    }
                    toggle = !toggle;
                }

        }

        oled_putString(1,1,(uint8_t *)"MONITOR",OLED_COLOR_BLACK,OLED_COLOR_WHITE);

        if (!btn1Press()) {
            currMode=Stable;
            printf("GOING STABLE\n");
            break;
        }

        //conditions
        if (ch==16) ch=0; //char rollover, 7seg

        if((getTicks()-currTime)>1000)
        {
            //7seg
            led7seg_setChar(numToChar(ch++), FALSE);

            //temp sensor polling for alert
            tempReading=temp_read()/10.0;

            //Display info on oLED, on '5' 'A' 'F'
            if((ch==6)||(ch==11)||ch==16)
            {
                //temp sensor
                printf("temp: %.1f\n", tempReading);
                sprintf(temp_sensor_value,"Temp: %.1f",tempReading);

                //light sensor
                lightReading = light_read();
                printf("light: %d\n", lightReading);
                sprintf(light_sensor_val, "Light: %d  " , lightReading);

                //accelerometer
                acc_read(&x, &y, &z);
                x = x+xoff;
                y = y+yoff;
                z = z+zoff;
                printf("acc: x:%d y:%d z:%d \n", x, y, z);
                sprintf(acc_sensor_value[0],"Acc:   x:%d  ", x);
                sprintf(acc_sensor_value[1],"     y:%d  ", y);
                sprintf(acc_sensor_value[2],"     z:%d  ", z);
                oled_putString(1,9,(uint8_t *)temp_sensor_value,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
                oled_putString(1,17,(uint8_t *)light_sensor_val,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
                oled_putString(1,25,(uint8_t *)acc_sensor_value[0],OLED_COLOR_BLACK,OLED_COLOR_WHITE);
                oled_putString(1,33,(uint8_t *)acc_sensor_value[1],OLED_COLOR_BLACK,OLED_COLOR_WHITE);
                oled_putString(1,41,(uint8_t *)acc_sensor_value[2],OLED_COLOR_BLACK,OLED_COLOR_WHITE);
            }

            //reconfig currTime
            currTime=getTicks();
        }

        //Timer0_Wait(1);
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

