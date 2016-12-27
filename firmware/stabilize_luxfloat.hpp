/*
   stabilize_multiwii.hpp : Class declaration for LuxFloat PID-based stablization

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

#ifdef __arm__
extern "C" {
#endif

    class StabilizeLuxFloat : public Stabilize {

        public:

            void init(class RC * _rc, class IMU * _imu);

            void update(bool armed);

            void resetIntegral(void);

        private:

           bool             deltaStateIsSet;
            biquad_t         deltaBiQuadState[3];
            int32_t          errorGyroI[3];
            float            errorGyroIf[3];
            bool             fullKiLatched;
            float            lastError[3];
            filterStatePt1_t yawPTermState;

            static int32_t getRcStickDeflection(int16_t * rcData, int32_t axis, uint16_t midrc);

            // indexed as roll / pitch / yaw

   };

#ifdef __arm__
} // extern "C"
#endif
