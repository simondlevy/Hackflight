/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "mockduino.h"

extern serialPort_t * telemport;

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

    setup();

    init_printf( NULL, _putc);
   
    // loopy
    while (1) 
        loop();
}
