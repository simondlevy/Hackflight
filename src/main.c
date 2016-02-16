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

#include "config.h"

static serialPort_t * telemport;

// gcc/GNU version
static void _putc(void *p, char c)
{
    (void)p; // avoid unused-argument warning
    serialWrite(telemport, c);
    while (!isSerialTransmitBufferEmpty(telemport));
}

int main(void)
{
    // from system_stm32f10x.c
    extern void SetSysClock(bool overclock);

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(CONFIG_EMF_AVOIDANCE);

    // set up initial conditions
    extern void setup();
    setup();
    
    extern serialPort_t * serialInit(uint32_t baudrate);
    telemport = serialInit(CONFIG_SERIAL_BAUDRATE);

    init_printf(NULL, _putc);

    extern void loop();
    while (1) 
        loop();
}
