/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"
#include "config.h"

core_t core;
extern rcReadRawDataPtr rcReadRawFunc;

// receiver read function
extern uint16_t pwmReadRawRC(uint8_t chan);

// from system_stm32f10x.c
void SetSysClock(bool overclock);

// gcc/GNU version
static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(core.mainport, c);
    while (!isSerialTransmitBufferEmpty(core.mainport));
}

int main(void)
{
    extern void setup(void);
    extern void loop(void);

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(0);

    systemInit();

    setup();

    init_printf(NULL, _putc);

    while (1) 
        loop();
}
