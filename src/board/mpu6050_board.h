#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "stm32f10x_conf.h"
#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"
#include "utils.h"

typedef enum HardwareRevision {
    NAZE32 = 1,                                         // Naze32 and compatible with 8MHz HSE
    NAZE32_REV5,                                        // Naze32 and compatible with 12MHz HSE
    NAZE32_SP,                                          // Naze32 w/Sensor Platforms
    NAZE32_REV6                                        // Naze32 rev6
} HardwareRevision;

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

typedef void (*sensorInitFuncPtr)(sensor_align_e align);   // sensor init prototype
typedef void (*sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype

typedef struct sensor_t {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;

#define I2C_DEVICE (I2CDEV_2)

#include "drv_adc.h"
#include "drv_i2c.h"

