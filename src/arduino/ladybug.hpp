/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

#include <hackflight_full.h>
#include <imu_alignment/rotate_270.h>
#include <mixers/fixedpitch/quadxbf.h>
#include <motor.h>
#include <stm32_clock.h>

static hackflight_t _hf;
        
static void ladybug_setup(rx_dev_funs_t * rxDeviceFuns)
{
    Wire.begin();
    delay(100);

    static uint8_t motorPins[4] = {13, 16, 3, 11};

    static anglePidConstants_t anglePidConstants = {
        1.441305,     // Rate Kp
        19.55048,     // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0}; // 3.0, // Level Kp

    motorInitBrushed(motorPins);

    stm32_startCycleCounter();

    // Always use Serial1 for receiver, no no need to specify
    hackflightInitFull(
            &_hf,
            rxDeviceFuns,
            SERIAL_PORT_NONE,
            &anglePidConstants,
            mixerQuadXbf,
            (void *)&motorPins,
            12,  // IMU interrupt pin
            imuRotate270,
            18); // LED pin
}

static void ladybug_loop(void)
{
    hackflightStep(&_hf);
}
