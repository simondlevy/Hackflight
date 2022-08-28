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

#include <stdint.h>
#include <stdbool.h>

#include <alignment/rotate270.h>
#include <clock.h>
#include <hackflight.h>
#include <mixers/fixedpitch/quadxbf.h>
#include <receivers/sbus.h>
#include <serial.h>

#include <imus/fusion/mpu6000.h>
#include <led/led.h>

#include "hardware_init.h"

int main(void)
{
    auto motorDevice = hardwareInit(Clock::PERIOD());

    static AnglePidController anglePid(
        1.441305,     // Rate Kp
        19.55048,     // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

    // static Mpu6000Imu imu(0); // dummy value for IMU interrupt pin
    static Mpu6000 imu(2000); // gyro scale DPS

    static SbusReceiver receiver(SERIAL_PORT_USART3);

    static Mixer mixer = QuadXbfMixer::make();

    static Stm32F4Led led(37); // pin

    static Hackflight hf(
            &receiver,
            &imu,
            imuRotate270,
            &anglePid,
            &mixer,
            motorDevice,
            &led);

    hf.begin();

    while (true) {

        hf.step();
    }

    return 0;
}
