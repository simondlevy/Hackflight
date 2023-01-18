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

#include <stdint.h>
#include <stdbool.h>

#include "esc.h"
#include "imu.h"
#include "core/vstate.h"
#include "receiver.h"

class Arming {

    private:

        static constexpr float MAX_ANGLE = 25;

        bool accDoneCalibrating;
        bool angleOkay;
        bool gyroDoneCalibrating;

    public:

        // Avoid repeated degrees-to-radians conversion
        const float maxAngle = Imu::deg2rad(MAX_ANGLE);

        bool gotFailsafe;
        bool haveSignal;
        bool isArmed;
        bool switchOkay;
        bool throttleIsDown;

        void attempt(Receiver & receiver, Esc & esc, const uint32_t usec)
        {
            static bool _doNotRepeat;

            if (receiver.aux1IsSet()) {

                if (ready()) {

                    if (isArmed) {
                        return;
                    }

                    if (!esc.isReady(usec)) {
                        return;
                    }

                    isArmed = true;
                }

            } else {

                disarm(esc);
            }

            if (!(isArmed || _doNotRepeat || !ready())) {
                _doNotRepeat = true;
            }
        }

        void disarm(Esc & esc)
        {
            if (isArmed) {
                esc.stop();
            }
            isArmed = false;
        }

        bool ready(void)
        {
            return 
                accDoneCalibrating &&
                angleOkay &&
                !gotFailsafe &&
                haveSignal &&
                gyroDoneCalibrating &&
                switchOkay &&
                throttleIsDown;
        }

        void updateFromImu(Imu & imu, VehicleState & vstate)
        {
            const auto imuIsLevel =
                fabsf(vstate.phi) < maxAngle && fabsf(vstate.theta) < maxAngle;

            angleOkay = imuIsLevel;

            gyroDoneCalibrating = !imu.gyroIsCalibrating();

            accDoneCalibrating = true; // XXX
        }

}; // class Arming
