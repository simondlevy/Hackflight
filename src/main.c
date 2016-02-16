/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#define I2C_DEVICE (I2CDEV_2)

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include "board/revision.h"
#include "board/printf.h"

#include "board/drv_adc.h"
#include "board/drv_i2c.h"
#include "board/drv_serial.h"
#include "board/drv_spi.h"
#include "board/drv_gpio.h"
#include "board/drv_system.h"
#include "board/drv_pwm.h"

#include "config.h"
#include "axes.h"
#include "mw.h"

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

    // determine hardware revision based on clock frequency
    int hw_revision = 0;
    if (hse_value == 8000000)
        hw_revision = NAZE32;
    else if (hse_value == 12000000)
        hw_revision = NAZE32_REV5;

    systemInit(hw_revision);

    // sleep for 100ms
    delay(100);

    if (spiInit() == SPI_DEVICE_MPU && hw_revision == NAZE32_REV5)
        hw_revision = NAZE32_SP;

    if (hw_revision != NAZE32_SP)
        i2cInit(I2C_DEVICE);

    adcInit(hw_revision);

    initSensors(hw_revision);

    // set up initial conditions
    setup();
    
    telemport = serialInit(CONFIG_SERIAL_BAUDRATE);

    init_printf(NULL, _putc);

    // loopy
    while (1) 
        loop();
}
