/*
   imu.hpp : IMU class header

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

#define CONFIG_ACC_LPF_FACTOR     4
#define CONFIG_ACCZ_DEADBAND      40
#define CONFIG_ACCXY_DEADBAND     40
#define CONFIG_ACCZ_LPF_CUTOFF    5.0F
#define CONFIG_GYRO_CMPF_FACTOR   600    
#define CONFIG_GYRO_CMPFM_FACTOR  250  
#define CONFIG_MORON_THRESHOLD     32

#ifdef __arm__
extern "C" {
#endif

    class IMU {
        
        private:

            Board * _board;

            uint16_t calibratingGyroCycles;
            uint16_t calibratingAccCycles;
            uint16_t acc1G;
            float    fcAcc;
            float    gyroScale;

        public:

            int16_t angle[3];
            int16_t gyroADC[3];

            void init(Board * board, uint16_t calibratingGyroCycles, uint16_t calibratingAccCycles);
            void update(uint32_t currentTime, bool armed, uint16_t & calibratingA, uint16_t & calibratingG);
    };

#ifdef __arm__
} // extern "C"
#endif
