#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "printf.h"
#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"
#include "utils.h"

#define I2C_DEVICE (I2CDEV_2)

typedef enum HardwareRevision {
    NAZE32 = 1,                                         // Naze32 and compatible with 8MHz HSE
    NAZE32_REV5,                                        // Naze32 and compatible with 12MHz HSE
    NAZE32_SP,                                          // Naze32 w/Sensor Platforms
    NAZE32_REV6                                        // Naze32 rev6
} HardwareRevision;

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

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
typedef void (*serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app
typedef uint16_t (*rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (*pidControllerFuncPtr)(void);                // pid controller function prototype

typedef struct sensor_t {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;

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



#define GYRO
#define ACC
#define BUZZER
#define LED0
#define LED1
#define INVERTER

#define I2C_DEVICE (I2CDEV_2)

#include "drv_adc.h"
#include "drv_i2c.h"
#include "drv_spi.h"
#include "drv_mpu6050.h"
#include "drv_pwm.h"
#include "drv_timer.h"
#include "drv_serial.h"
#include "drv_uart.h"

#ifdef INV_GPIO
#define INV_OFF                  digitalLo(INV_GPIO, INV_PIN);
#define INV_ON                   digitalHi(INV_GPIO, INV_PIN);
#else
#define INV_OFF                 ;
#define INV_ON                  ;
#endif
