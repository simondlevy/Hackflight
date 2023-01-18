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

#include "imu.h"
#include "core/vstate.h"

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
