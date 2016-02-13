#pragma once

// for roundf()
#define __USE_C99_MATH

#define I2C_DEVICE (I2CDEV_2)

#include <stdbool.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "drv_system.h"         // timers, delays, etc
#include "drv_gpio.h"

