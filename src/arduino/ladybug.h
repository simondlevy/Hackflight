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

#include <alignment/rotate0.h>
#include <core/mixers/fixedpitch/quadxbf.h>
#include <hackflight.h>
#include <imus/usfs/usfs.h>
#include <leds/arduino.h>
#include <escs/brushed.h>
#include <stm32clock.h>

#include <vector>
using namespace std;

static UsfsImu _imu(12); // interrupt pin

static AnglePidController _anglePid(
        1.441305,     // Rate Kp
        19.55048,     // Rate Ki
        0.021160,     // Rate Kd
        0.0165048,    // Rate Kf
        0.0); // 3.0; // Level Kp


static Mixer _mixer = QuadXbfMixer::make();

static ArduinoLed _led(18); // pin

static vector<uint8_t> _motorPins = {13, 16, 3, 11};

static vector<PidController *> _pidControllers = {&_anglePid};

static ArduinoBrushedEsc _esc(_motorPins);

class LadybugFc : public Hackflight {

    public:

        LadybugFc(Receiver & receiver) 

            : Hackflight(
                    receiver,
                    _imu,
                    imuRotate0,
                    _pidControllers,
                    _mixer,
                    _esc,
                    _led)
        {
        }

        void begin(void)
        {
            Wire.begin();
            delay(100);

            stm32_startCycleCounter();

            Hackflight::begin();
        }
};
