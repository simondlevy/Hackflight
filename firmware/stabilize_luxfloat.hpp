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

            // indexed as roll / pitch / yaw
            const float   PID_P_f[3] = {5.0f, 6.5f, 9.3f}; 
            const float   PID_I_f[3] = {1.0f, 1.5f, 1.75f};
            const float   PID_D_f[3] = {0.11f, 0.14f, 0.0f};
            const uint8_t PID_WEIGHT[3] = {100, 100, 100};
            const uint8_t PID_CONTROL_RATES[3] = {90, 90, 90};
            const uint8_t PID_ANGLE_TRIMS_RAW[3] = {0, 0, 0};

            const uint8_t ESCWriteDenominator = 1; // ESC Write at 1khz
            const uint16_t gyroSamplePeriod = 125; // XXX estimated
            const uint32_t targetESCwritetime = gyroSamplePeriod*ESCWriteDenominator;
            const float dT = (float)targetESCwritetime * 0.000001f;

            const float   KD_ATTENUATION_BREAK = 0.25f;

            bool             deltaStateIsSet;
            biquad_t         deltaBiQuadState[3];
            int32_t          errorGyroI[3];
            float            errorGyroIf[3];
            bool             fullKiLatched;
            float            lastError[3];
            filterStatePt1_t yawPTermState;

            static int32_t getRcStickDeflection(int16_t * rcData, int32_t axis, uint16_t midrc);
    }; 

#ifdef __arm__
} // extern "C"
#endif
