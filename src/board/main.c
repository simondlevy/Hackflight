/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include "printf.h"
#include "drv_serial.h"
#include "drv_uart.h"
#include "drv_gpio.h"
#include "drv_system.h"

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
    // Do our best to mock up Arduino approach
    extern void setup();
    extern void loop();
    extern void serialInit(serialPort_t * telemport);

    // from system_stm32f10x.c
    extern void SetSysClock(bool overclock);

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(false);

    // set up initial conditions
    setup();
    
    // set up a telemetry port
    telemport = uartOpen(USART1, NULL, 115200, MODE_RXTX);
    serialInit(telemport);

    // intitialize for printing over this port
    init_printf(NULL, _putc);

    while (1) 
        loop();
}
