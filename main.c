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
#include <String.h>

#include "led7seg.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "light.h"
#include "temp.h"
#include "joystick.h"

typedef enum {
    Monitor, Stable,
} Mode;

Mode currMode = Stable;

static const int LIGHT_LOW_WARNING = 50;
static const float TEMP_HIGH_WARNING = 45.0;
static const int MOVEMENT_THRESHOLD = 4;
static const int DEBOUNCE_TIME = 500;
float temp_adjust = 0.0;
int light_adjust = 0;

Bool alert;

uint16_t uart_count = 0;

int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;

int8_t x = 0;
int8_t y = 0;
int8_t z = 0;

int8_t xPrev = 0;
int8_t yPrev = 0;
int8_t zPrev = 0;
uint32_t lastpress = 0;

int lightReading;
float tempReading;

volatile uint32_t msTicks; // counter for 1ms SysTicks

// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
    msTicks++;
}

static void init_ssp(void) {
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

static void init_i2c(void) {
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

static void init_GPIO(void) {
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
    PinCfg.Pinnum = 15;
    PINSEL_ConfigPin(&PinCfg); // joystick
    PinCfg.Pinnum = 16;
    PINSEL_ConfigPin(&PinCfg); // joystick
    PinCfg.Pinnum = 17;
    PINSEL_ConfigPin(&PinCfg); // joystick
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
    PinCfg.Pinnum = 3;
    PINSEL_ConfigPin(&PinCfg); //joystick
    PinCfg.Pinnum = 4;
    PINSEL_ConfigPin(&PinCfg); //joystick

    GPIO_SetDir(2, (1 << 0), 1);
    GPIO_SetDir(0, (1 << 26), 1);
    GPIO_SetDir(2, (1 << 1), 1);

}

void pinsel_uart3(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

static void init_uart(void) {

    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;

    pinsel_uart3();

    UART_Init(LPC_UART3, &uartCfg);

    UART_TxCmd(LPC_UART3, ENABLE);
}

static uint32_t getTicks() {
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
    xPrev = x;
    yPrev = y;
    zPrev = z;
    acc_read(&x, &y, &z);
    x = x + xoff;
    y = y + yoff;
    z = z + zoff;
    xPrev = x;
    yPrev = y;
    zPrev = z;

}

static void init_all() {
    init_i2c();
    init_ssp();
    init_GPIO();

    init_uart();

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

    LPC_GPIOINT ->IO2IntEnF |= 1 << 5; //GPIO interrupt P2.5 for light sensor
    NVIC_EnableIRQ(EINT3_IRQn); // enable EINT3 interrupt

    oled_clearScreen(OLED_COLOR_WHITE);
}

static void drawOled(uint8_t joyState)
{
    static int wait = 0;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        temp_adjust = 0.0;
        light_adjust = 0;
        return;
    }

    if (wait++ < 2)
        return;

    wait = 0;

    if ((joyState & JOYSTICK_UP) != 0 && temp_adjust < 20.0 ) {
        temp_adjust += 0.1;
    }

    if ((joyState & JOYSTICK_DOWN) != 0 && temp_adjust > -20.0 ) {
        temp_adjust -= 0.1;
    }

    if ((joyState & JOYSTICK_RIGHT) != 0 && light_adjust < 20) {
        light_adjust += 1;
    }

    if ((joyState & JOYSTICK_LEFT) != 0 && light_adjust > -20) {
        light_adjust -= 1;
    }
}

void EINT3_IRQHandler(void) {
//  int i;
    // Determine whether GPIO Interrupt P2.5 has occurred
    if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1) {
        //printf("LOW LIGHT WARNING\n");
        lightReading = light_read();
        light_clearIrqStatus();
        LPC_GPIOINT ->IO2IntClr |= 1 << 5;
    }
}

int btn1Press() { //SW4
    return (GPIO_ReadValue(1) >> 31) & 0x01;
}

int btn2Press() { //SW3
    return (GPIO_ReadValue(0) >> 4) & 0x01;
}

void UART_send_improved(char* msg) {
    int len =strlen(msg);
    msg[len++]='\n';
    msg[len++]='\r';
    msg[len]=0;
    UART_Send(LPC_UART3, (uint8_t *)msg, (uint32_t)len, BLOCKING);
}

void acc_read_improved(int8_t *x, int8_t *y, int8_t *z) {
    xPrev = *x;
    yPrev = *y;
    zPrev = *z;
    acc_read(x, y, z);
    *x = *x + xoff;
    *y = *y + yoff;
    *z = *z + zoff;
}

int isMoving() {
    return (abs(xPrev - x) / MOVEMENT_THRESHOLD
            || abs(yPrev - y) / MOVEMENT_THRESHOLD
            || abs(zPrev - z) / MOVEMENT_THRESHOLD);
    //return 1; // stub
}

void stableMode() {

    while (currMode == Stable) {
        oled_clearScreen(OLED_COLOR_BLACK); // clear oled
        led7seg_setChar(' ', FALSE); // 7seg switch off
        rgb_setLeds(4); // rgb switch off
        alert = FALSE; // turn off warnings

        if (!btn1Press() && ((getTicks() - lastpress) > DEBOUNCE_TIME)) {
            currMode = Monitor;
            lastpress = getTicks();
            break;
        }
    }
}

void monitorMode() {

    uint8_t ch = 0;

    uint32_t currTime = getTicks();
    uint32_t blinkTime = getTicks();
    uint8_t state = 0;

    int a = 0, b = 0;

    char temp_sensor_value[40];
    char light_sensor_val[40];
    char acc_sensor_value[3][40];
    char thresholds[40];
    Bool toggle = FALSE;
    Bool displayed = FALSE;
    char monitorMsg[40]="Entering MONITOR mode.";


    acc_setup();

    oled_clearScreen(OLED_COLOR_WHITE);

    UART_send_improved(monitorMsg);

    tempReading = temp_read() / 10.0;
    lightReading = light_read();
    acc_read_improved(&x, &y, &z);

    while (currMode == Monitor) {
        if (tempReading > TEMP_HIGH_WARNING + temp_adjust) {
            alert = TRUE;
            a = 1;
        }
        if ((lightReading < LIGHT_LOW_WARNING + ligh_adjust) && isMoving()) {
            alert = TRUE;
            b = 2;
        }

        if (alert) {
            if ((getTicks() - blinkTime) > 166) {
                if (toggle) {
                    rgb_setLeds(4 + a + b);
                } else {
                    rgb_setLeds(4);
                }
                toggle = !toggle;
                blinkTime = getTicks();
            }
        }

        oled_putString(1, 1, (uint8_t *) "MONITOR", OLED_COLOR_BLACK,
                OLED_COLOR_WHITE);

        if (!btn1Press() && ((getTicks() - lastpress) > DEBOUNCE_TIME)) {
            currMode = Stable;
            lastpress = getTicks();
            break;
        }

        //Display info on oLED, on '5' 'A' 'F'
        if (((ch == 6) || (ch == 11) || (ch == 16)) && (!displayed)) {
            //temp sensor
            tempReading = temp_read() / 10.0;
            //printf("temp: %.1f\n", tempReading);
            sprintf(temp_sensor_value, "Temp: %.1f", tempReading);

            //light sensor
            lightReading = light_read();
            //printf("light: %d\n", lightReading);
            sprintf(light_sensor_val, "Light: %d  ", lightReading);

            //accelerometer
            acc_read_improved(&x, &y, &z);

            //printf("acc: x:%d y:%d z:%d \n", x, y, z);
            sprintf(acc_sensor_value[0], "Acc: x:%d  ", x);
            sprintf(acc_sensor_value[1], "     y:%d  ", y);
            sprintf(acc_sensor_value[2], "     z:%d  ", z);

            oled_putString(1, 9, (uint8_t *) temp_sensor_value,
                    OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1, 17, (uint8_t *) light_sensor_val,
                    OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1, 25, (uint8_t *) acc_sensor_value[0],
                    OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1, 33, (uint8_t *) acc_sensor_value[1],
                    OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1, 41, (uint8_t *) acc_sensor_value[2],
                    OLED_COLOR_BLACK, OLED_COLOR_WHITE);

            displayed = TRUE;
        }

        //Joystick
        state = joystick_read();
        if (state != 0)
            drawOled(state);


        char msgDisplay[99];
        char msgFire[40];
        char msgDarkMovement[40];
        //conditions
        //Counter reaches number F, UART sends message.
        if (ch == 16) {
            ch = 0; //char rollover, 7seg

            if (a == 1) {
                sprintf(msgFire, "Fire was Detected.");
                UART_send_improved(msgFire);
            }
            if (b == 2){
                sprintf(msgDarkMovement, "Movement in darkness was Detected.");
                UART_send_improved(msgDarkMovement);
            }

            sprintf(msgDisplay, "%03d_-_T%03.1f_L%04d_AX%02d_AY%02d_AZ%02d",
                                    uart_count++, tempReading, lightReading, x, y, z);
            printf(msgDisplay);
            UART_send_improved(msgDisplay);

        }

        if ((getTicks() - currTime) > 1000) {
            //resets display token
            displayed = FALSE;
            //7seg
            led7seg_setChar(numToChar(ch++), FALSE);
            sprintf(thresholds, "T/L: %.1f /%3d", TEMP_HIGH_WARNING + temp_adjust, LIGHT_LOW_WARNING + light_adjust);
            oled_putString(1, 51, (uint8_t *) thresholds, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

            //reconfig currTime
            currTime = getTicks();
        }
    }
}

int main(void) {

//sysTick
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1)
            ;  // Capture error
    }

    init_all();

    while (1) {
        if (currMode == Monitor)
            monitorMode();
        if (currMode == Stable)
            stableMode();
    }

}

void check_failed(uint8_t *file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
        ;
}

