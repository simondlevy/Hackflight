/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "breezystm32.h"

serialPort_t * Serial1;

// from system_stm32f10x.c
extern void SetSysClock(bool overclock);

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    serialWrite(Serial1, c);

    while (!isSerialTransmitBufferEmpty(Serial1));
}

int main(void)
{
    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(0);

    systemInit();

    // Suport just one baud rate for now
    Serial1 = uartOpen(USART1, NULL, 115200, MODE_RXTX);

    setup();

    init_printf( NULL, _putc);
   
    // loopy
    while (1) 
        loop();
}
