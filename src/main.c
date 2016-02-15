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

int     hw_revision = 0;
extern  rcReadRawDataPtr rcReadRawFunc;
uint8_t useSmallAngle;
uint8_t armed;

// receiver read function
extern uint16_t pwmReadRawRC(uint8_t chan);

// from system_stm32f10x.c
void SetSysClock(bool overclock);

// gcc/GNU version
static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(telemport, c);
}

static void activateConfig(void)
{
    uint8_t i;
    for (i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + CONFIG_RC_EXPO_8 * (i * i - 25)) * i * (int32_t)CONFIG_RC_RATE_8 / 2500;

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - CONFIG_THR_MID_8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - CONFIG_THR_MID_8;
        if (tmp < 0)
            y = CONFIG_THR_MID_8;
        lookupThrottleRC[i] = 10 * CONFIG_THR_MID_8 + tmp * (100 - CONFIG_THR_EXPO_8 + 
                (int32_t)CONFIG_THR_EXPO_8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = CONFIG_MINTHROTTLE + (int32_t)(CONFIG_MAXTHROTTLE - CONFIG_MINTHROTTLE) * 
            lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

int main(void)
{
    uint8_t i;

    armed = 0;

    // Configure clock, this figures out HSE for hardware autodetect
    SetSysClock(CONFIG_EMF_AVOIDANCE);

    // determine hardware revision based on clock frequency
    if (hse_value == 8000000)
        hw_revision = NAZE32;
    else if (hse_value == 12000000)
        hw_revision = NAZE32_REV5;

    systemInit();
    init_printf(NULL, _putc);

    // sleep for 100ms
    delay(100);

    activateConfig();

    if (spiInit() == SPI_DEVICE_MPU && hw_revision == NAZE32_REV5)
        hw_revision = NAZE32_SP;

    if (hw_revision != NAZE32_SP)
        i2cInit(I2C_DEVICE);

    adcInit();

    initSensors();

    LED1_ON;
    LED0_OFF;
    for (i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;

    imuInit(); 
    mixerInit(); 

    serialInit(CONFIG_SERIAL_BAUDRATE);

    pwmInit(CONFIG_FAILSAFE_DETECT_THRESHOLD, CONFIG_PWM_FILTER, CONFIG_USE_CPPM, CONFIG_MOTOR_PWM_RATE,
            CONFIG_FAST_PWM, CONFIG_PWM_IDLE_PULSE);

    // configure PWM/CPPM read function and max number of channels
    // these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;
    rcReadRawFunc = pwmReadRawRC;

    previousTime = micros();

    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;

    // trigger accelerometer calibration requirement
    useSmallAngle = 1;
    
    // loopy
    while (1) {
        loop();
    }
}
