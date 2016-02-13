/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

// for roundf()
#define __USE_C99_MATH

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
#include "board/drv_system.h"         // timers, delays, etc
#include "board/drv_gpio.h"
#include "board/drv_px4flow.h"
#include "board/drv_lidarlite.h"
#include "board/drv_serial.h"
#include "utils.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f
#define RAD    (M_PI / 180.0f)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

typedef uint16_t (*rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (*pidControllerFuncPtr)(void);                // pid controller function prototype

// Hardware definitions and GPIO
// Target definitions (NAZE, ... are same as in Makefile
#if defined(NAZE)

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Buzzer)
#define INV_PIN     Pin_2 // PB2 (BOOT1) abused as inverter select GPIO
#define INV_GPIO    GPIOB
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

#define GYRO
#define ACC
#define BUZZER
#define LED0
#define LED1
#define INVERTER

#define I2C_DEVICE (I2CDEV_2)

#include "board/drv_adc.h"
#include "board/drv_i2c.h"
#include "board/drv_spi.h"

#include "drv_pwm.h"
#include "drv_timer.h"
#include "drv_uart.h"

#endif /* all conditions */

// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);
#else
#define LED0_TOGGLE
#define LED0_OFF
#define LED0_ON
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);
#else
#define LED1_TOGGLE
#define LED1_OFF
#define LED1_ON
#endif

#ifdef BEEP_GPIO
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 systemBeep(false);
#define BEEP_ON                  systemBeep(true);
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif

#ifdef INV_GPIO
#define INV_OFF                  digitalLo(INV_GPIO, INV_PIN);
#define INV_ON                   digitalHi(INV_GPIO, INV_PIN);
#else
#define INV_OFF                 ;
#define INV_ON                  ;
#endif
