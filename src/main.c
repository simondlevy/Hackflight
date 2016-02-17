/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include "board/printf.h"

#include "board/drv_serial.h"
#include "board/drv_gpio.h"
#include "board/drv_system.h"

#include "sensors.h"
#include "config.h"
#include "axes.h"
#include "mw.h"

int     hw_revision = 0;
extern  rcReadRawDataPtr rcReadRawFunc;
uint8_t useSmallAngle;
uint8_t armed;

// receiver read function
extern uint16_t pwmReadRawRC(uint8_t chan);

// from system_stm32f10x.c
void SetSysClock(bool overclock);

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    serialWrite(telemport, c);

    while (!isSerialTransmitBufferEmpty(telemport));
}

int main(void)
{
    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(0);

    setup();

    init_printf( NULL, _putc);

   
    // loopy
    while (1) 
        loop();
}
