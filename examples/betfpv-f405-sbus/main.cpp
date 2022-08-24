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

#include <clock.h>
#include <datatypes.h>
#include <hackflight_full.h>
#include <imus/fusion.h>
#include <imu_alignment/rotate_270.h>
#include <mixers/fixedpitch/quadxbf.h>
#include <receivers/sbus.h>
#include <serial_device.h>

#include <stm32f4_cpp/led.h>

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

    static FusionImu imu(0); // dummy value for IMU interrupt pin

    static SbusReceiver receiver(SERIAL_PORT_USART3);

    static QuadXbfMixer mixer; 

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

void HardFault_Handler(void)
{
}
