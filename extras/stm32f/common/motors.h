/*
   motors.h : STM32F3 motor support

   Copyright (C) 2018 Simon D. Levy 

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

// Here we put code that interacts with Cleanflight
extern "C" {

    // Cleanflight includes
#include "platform.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "target.h"
#include "stm32f30x.h"

    void brushed_motors_init(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);

    void brushless_motors_init(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);

    void motor_write(uint8_t index, float value);

} // extern "C"
