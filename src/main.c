/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "board.h"
#include "mw.h"
#include "telemetry_common.h"
#include "config.h"

core_t core;
int hw_revision = 0;
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
    bool sensorsOK = false;

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

    // drop out any sensors that don't seem to work, init all the others. halt if gyro is dead.
    sensorsOK = sensorsAutodetect();

    // if gyro was not detected due to whatever reason, we give up now.
    if (!sensorsOK)
        failureMode(3);

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

    pwmInit();

    // configure PWM/CPPM read function and max number of channels
    // these, if enabled
    for (i = 0; i < RC_CHANS; i++)
        rcData[i] = 1502;
    rcReadRawFunc = pwmReadRawRC;
    core.numRCChannels = MAX_INPUTS;

    previousTime = micros();
    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
    // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

    // trigger accelerometer calibration requirement
    useSmallAngle = 1;
    
    px4flow_available = initPX4Flow();
    //lidarlite_available = initLidarLite();

    // loopy
    while (1) {
        loop();
    }
}
