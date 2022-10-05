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

#include <alignment/rotate270.h>
#include <core/clock.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <escs/dshot/bitbang.h>
#include <hackflight.h>
#include <imus/mock.h>
#include <leds/stm32f4.h>
#include <tasks/receivers/sbus.h>
#include <serial.h>

#include <spi.h>

#include <io_def_generated.h>

#include "hardware_init.h"

#include <vector>
using namespace std;

static const uint8_t LED_PIN = 0x25;

int main(void)
{
    hardwareInit();

    static AnglePidController anglePid(
        1.441305,     // Rate Kp
        48.8762,      // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp

    static MockImu imu;

    vector<PidController *> pids = {&anglePid};

    vector<uint8_t> motorPins = {0x20, 0x21, 0x13, 0x12};

    static SbusReceiver rx(SERIAL_PORT_USART3);

    static Mixer mixer = QuadXbfMixer::make();

    static DshotBitbangEsc esc(motorPins);

    static Stm32F4Led led(LED_PIN);

    static Hackflight hf(rx, imu, imuRotate270, pids, mixer, esc, led);

    hf.begin();

    while (true) {

        hf.step();
    }

    return 0;
}
