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

//#include "stdio.h"

#include "led7seg.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"

static uint8_t barPos = 2;

volatile uint32_t msTicks; // counter for 1ms SysTicks

// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
    msTicks++;
}

static void moveBar(uint8_t steps, uint8_t dir)
{
    uint16_t ledOn = 0;

    if (barPos == 0)
        ledOn = (1 << 0) | (3 << 14);
    else if (barPos == 1)
        ledOn = (3 << 0) | (1 << 15);
    else
        ledOn = 0x07 << (barPos-2);

    barPos += (dir*steps);
    barPos = (barPos % 16);

    //pca9532_setLeds(ledOn, 0xffff);
}


static void drawOled(uint8_t joyState)
{
    static int wait = 0;
    static uint8_t currX = 48;
    static uint8_t currY = 32;
    static uint8_t lastX = 0;
    static uint8_t lastY = 0;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        oled_clearScreen(OLED_COLOR_BLACK);
        return;
    }

    if (wait++ < 3)
        return;

    wait = 0;

    if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
        currY--;
    }

    if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT-1) {
        currY++;
    }

    if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH-1) {
        currX++;
    }

    if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
        currX--;
    }

    if (lastX != currX || lastY != currY) {
        oled_putPixel(currX, currY, OLED_COLOR_WHITE);
        lastX = currX;
        lastY = currY;
    }
}


#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);




static uint32_t notes[] = {
        2272, // A - 440 Hz -6.
        2024, // B - 494 Hz -7.
        3816, // C - 262 Hz -1
        3401, // D - 294 Hz -2
        3030, // E - 330 Hz -3
        2865, // F - 349 Hz -4
        2551, // G - 392 Hz -5
        2152, // H - 392 Hz -7^b
        1136, // a - 880 Hz -6
        1012, // b - 988 Hz -7
        1912, // c - 523 Hz -1^
        1703, // d - 587 Hz -2^
        1517, // e - 659 Hz -3^
        1432, // f - 698 Hz -4^
        1275, // g - 784 Hz -5^
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }

    }
    else {
        Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'H')
        return notes[ch - 'A'];

    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 8];

    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;

    /* number of ms */

    return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {                  //*song is pointer, pointing to start of string;
        led7seg_setChar(*song, FALSE);      //"\0" this character indicates end of string;
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        //delay32Ms(0, pause);
        Timer0_Wait(pause);

    }
}
static uint8_t * song = (uint8_t*)"e1,e2.e2,c1,e2,g4_G4,c3,G2,E2,A4,B2,H2,A4,G2,e2,g2,a2_f2,g2,e2+c1_d2,B4,";
//static uint8_t * song = (uint8_t*)"C2.C2,D4,C4,F4,E8,";
//static uint8_t * song = (uint8_t*)"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,H2.H2,A4,F4,G4,F8,";
        //"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";



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
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 4;
    PINSEL_ConfigPin(&PinCfg);

    //PINSEL_CFG_Type PinCfg;
    //PinCfg.Funcnum = 0;
    //PinCfg.OpenDrain = 0;
    //PinCfg.Pinmode = 0;
    //PinCfg.Portnum = 2;
    //PinCfg.Pinnum = 10;
    //PINSEL_ConfigPin(&PinCfg);
}


int main (void) {


    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;

    int8_t x = 0;

    int8_t y = 0;
    int8_t z = 0;
    uint8_t dir = 1;
    uint8_t wait = 0;

    uint8_t state    = 0;

    uint8_t btn1 = 0;
    uint8_t btn2 = 0;

    Bool tone_toggle = FALSE;

    uint8_t ch = 48;
    uint32_t ledNum = 1;
    uint8_t rgbNum = 0;

    uint32_t btn2_time = msTicks;
    uint32_t led7seg_time = msTicks;
    uint32_t ledArr_time = msTicks;
    uint32_t ledRGB_time = msTicks;

    uint32_t lightReading;


//sysTick
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);  // Capture error
    }

    init_i2c();
    init_ssp();
    init_GPIO();
    rgb_init();

    pca9532_init();
    joystick_init();
    acc_init();
    light_init();
    oled_init();
    led7seg_init();


    //rgb_setLeds(0);

    //pca9532_setLeds(65280,0);

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z;

    /* ---- Speaker ------> */

    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    /* <---- Speaker ------ */

    moveBar(1, dir);
    oled_clearScreen(OLED_COLOR_BLACK);

    while (1)
    {

        /* ####### Accelerometer and LEDs  ###### */
        /* # */

        acc_read(&x, &y, &z);
        x = x+xoff;
        y = y+yoff;
        z = z+zoff;

        if (y < 0) {
            dir = 1;
            y = -y;
        }
        else {
            dir = -1;
        }

        if (y > 1 && wait++ > (40 / (1 + (y/10)))) {
            //moveBar(1, dir);
            wait = 0;
        }


        /* # */
        /* ############## Light Sensor #################### */
        lightReading = light_read();

        /* # */
        /* ############################################# */


        /* ####### Joystick and OLED  ###### */
        /* # */

        state = joystick_read();
        if (state != 0)
            drawOled(state);

        /* #rgb_LED */
        /* ############################################# */



        /* ############ Trimpot and RGB LED  ########### */
        /* # */


        if (ch==58)
        {
            ch=65;
        }
        if (ch==71)
        {
            ch=48;
        }

        if (ledNum>=0x00010000)
        {
            ledNum=1;
        }

        if(rgbNum>=0x02)
        {
            rgbNum=0;
        }


        if((msTicks-led7seg_time)>1000)
        {
            led7seg_setChar(ch, FALSE);
            ch++;
            led7seg_time=msTicks;
        }

        if((msTicks-ledArr_time)>1000)
        {
            pca9532_setLeds(ledNum,0xffff);
            ledNum*=2;
            ledArr_time=msTicks;
        }

        if((msTicks-ledRGB_time)>2000)
        {
            rgb_setLeds(rgbNum++);
            //rgbNum++;
            ledRGB_time=msTicks;
        }


        btn2 = (GPIO_ReadValue(0) >> 4) & 0x01;


        if((btn2 == 0) && ((msTicks-btn2_time)>200))
        {
            tone_toggle=~tone_toggle;
            btn2_time=msTicks;
        }

        if(tone_toggle)
        {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(1432 / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(1432 / 2);
        }
        else
        {
            NOTE_PIN_LOW();
        }


        btn1 = (GPIO_ReadValue(1) >> 31) & 0x01;

        if (btn1 == 0)
        {
            playSong(song);

        }


        Timer0_Wait(1);
    }


}

void check_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while(1);
}

