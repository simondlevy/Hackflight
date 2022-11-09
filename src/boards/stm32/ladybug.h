/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "alignment/rotate0.h"
#include "boards/stm32.h"
#include "escs/brushed.h"
#include "imus/real/usfs.h"

class LadybugBoard : public Stm32Board {

    private:

        vector<uint8_t> motorPins = {13, 16, 3, 11};

        UsfsImu imu;

        BrushedEsc esc = BrushedEsc(motorPins);

    public:

        static const uint8_t LED_PIN = 0x12;

        LadybugBoard( Receiver & rx, vector<PidController *> & pids, Mixer & mixer)
            : Stm32Board(rx, imu, imuRotate0, pids, mixer, esc, LED_PIN)
        {
        }

        void handleInterrupt(void)
        {
            imu.handleInterrupt();
        }

        static const uint8_t IMU_INTERRUPT_PIN = 0x0C;

}; // class LadybugBoard
