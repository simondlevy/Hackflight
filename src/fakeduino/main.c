/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include "printf.h"

#include "drv_serial.h"
#include "drv_system.h"

#include "fakeduino.h"

extern serialPort_t * telemport;
void serialInit(uint32_t baudrate);

// from system_stm32f10x.c
extern void SetSysClock(bool overclock);

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

    systemInit();

    serialInit(115200);

    setup();

    init_printf( NULL, _putc);
   
    // loopy
    while (1) 
        loop();
}
