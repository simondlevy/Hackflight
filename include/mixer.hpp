/*
   mixer.hpp : Mixer class header

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

#include "board.hpp"
#include "stabilize.hpp"

#include <cstring>

namespace hf {

class Mixer {

public:

    // This is set by MSP
    int16_t  motorsDisarmed[4];

    void init(const PwmConfig& _pwmConfig, RC * _rc, Stabilize * _stabilize, Board * _board);
    void update(bool armed);

private:

    PwmConfig pwmConfig;
    RC        * rc;
    Stabilize * stabilize;

    Board * board;

    // Custom mixer data per motor
    typedef struct motorMixer_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } motorMixer_t;

	motorMixer_t mixerQuadX[4];
};


/********************************************* CPP ********************************************************/

void Mixer::init(const PwmConfig& _pwmConfig, RC * _rc, Stabilize * _stabilize, Board * _board)
{
    mixerQuadX[0] = { +1.0f, -1.0f,  +1.0f, -1.0f };    // right rear
    mixerQuadX[1] = { +1.0f, -1.0f,  -1.0f, +1.0f };    // right front
    mixerQuadX[2] = { +1.0f, +1.0f,  +1.0f, +1.0f };    // left rear
    mixerQuadX[3] = { +1.0f, +1.0f,  -1.0f, -1.0f };    // left front

    memcpy(&pwmConfig, &_pwmConfig, sizeof(PwmConfig));

    stabilize = _stabilize;
    rc = _rc;
    board = _board;

    // set disarmed motor values
    for (uint8_t i = 0; i < 4; i++)
        motorsDisarmed[i] = pwmConfig.min;
}

void Mixer::update(bool armed)
{
    int16_t motors[4];

    for (uint8_t i = 0; i < 4; i++) {
        motors[i] = (int16_t)
        (rc->command[DEMAND_THROTTLE]   * mixerQuadX[i].throttle + 
         stabilize->axisPID[AXIS_PITCH] * mixerQuadX[i].pitch + 
         stabilize->axisPID[AXIS_ROLL]  * mixerQuadX[i].roll - 
         stabilize->axisPID[AXIS_YAW]   * mixerQuadX[i].yaw);
    }

    int16_t maxMotor = motors[0];

    for (uint8_t i = 1; i < 4; i++)
        if (motors[i] > maxMotor)
            maxMotor = motors[i];

    for (uint8_t i = 0; i < 4; i++) {

        // This is a way to still have good gyro corrections if at least one motor reaches its max
        if (maxMotor > pwmConfig.max) {
            motors[i] -= maxMotor - pwmConfig.max;
        }

        motors[i] = constrain(motors[i], pwmConfig.min, pwmConfig.max);

        // Avoid sudden motor jump from right yaw while arming
        if (rc->throttleIsDown()) {
            motors[i] = pwmConfig.min;
        } 

        // This is how we can spin the motors from the GCS
        if (!armed) {
            motors[i] = motorsDisarmed[i];
        }
    }

    for (uint8_t i = 0; i < 4; i++) {
        board->writeMotor(i, motors[i]);
    }
}

} // namespace
