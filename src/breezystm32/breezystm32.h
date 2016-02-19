#pragma once

#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"

#include "drv_adc.h"
#include "drv_i2c.h"
#include "drv_serial.h"
#include "drv_gpio.h"
#include "drv_system.h"
#include "drv_pwm.h"
#include "drv_spi.h"
#include "drv_uart.h"

#include "printf.h"

extern serialPort_t * Serial1;

void setup(void);
void loop(void);
