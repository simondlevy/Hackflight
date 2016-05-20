/*
   pid.hpp : PID class header

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

#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */

#ifdef __arm__
extern "C" {
#endif

    class PID {

        private:

            /* For 130mm quad */
            static const uint8_t CONFIG_LEVEL_P          = 40;
            static const uint8_t CONFIG_LEVEL_I          = 2;

            static const uint8_t CONFIG_RATE_PITCHROLL_P = 20;
            static const uint8_t CONFIG_RATE_PITCHROLL_I = 15;
            static const uint8_t CONFIG_RATE_PITCHROLL_D = 11;

            static const uint8_t CONFIG_YAW_P            = 40;
            static const uint8_t CONFIG_YAW_I            = 20;

            /* For 250mm quad 
            static const uint8_t CONFIG_LEVEL_P          = 90;
            static const uint8_t CONFIG_LEVEL_I          = 10;

            static const uint8_t CONFIG_RATE_PITCHROLL_P = 40;
            static const uint8_t CONFIG_RATE_PITCHROLL_I = 30;
            static const uint8_t CONFIG_RATE_PITCHROLL_D = 23;

            static const uint8_t CONFIG_YAW_P            = 85;
            static const uint8_t CONFIG_YAW_I            = 45;
            */

            uint8_t rate_p[3];
            uint8_t rate_i[3];
            uint8_t rate_d[3];

            int16_t lastGyro[3];
            int32_t delta1[3]; 
            int32_t delta2[3];
            int32_t errorGyroI[3];
            int32_t errorAngleI[2];

        public:

            int16_t axisPID[3];

            void init(void);

            void update(RC * rc, IMU * imu);

            void resetIntegral(void);
    }; 

#ifdef __arm__
} // extern "C"
#endif
