/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>

#include "stm32f10x_conf.h"
#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"

typedef enum HardwareRevision {
    NAZE32 = 1,                                         // Naze32 and compatible with 8MHz HSE
    NAZE32_REV5,                                        // Naze32 and compatible with 12MHz HSE
    NAZE32_SP,                                          // Naze32 w/Sensor Platforms
    NAZE32_REV6                                        // Naze32 rev6
} HardwareRevision;

typedef void (*baroOpFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)

typedef struct baro_t {
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroOpFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroOpFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;


#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

#define I2C_DEVICE (I2CDEV_2)

#include "drv_i2c.h"

