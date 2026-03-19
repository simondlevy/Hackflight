/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <MPU6050.h>

#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>

namespace hf {

    class IMU {

        private:

            static constexpr uint8_t GRANGE = MPU6050_GYRO_FS_2000;
            static constexpr uint8_t ARANGE = MPU6050_ACCEL_FS_16;

        public:

            void begin()
            {
                Wire.begin();

                Wire.setClock(1000000); 

                _mpu6050.initialize();

                if (!_mpu6050.testConnection()) {
                    Debugger::reportForever("MPU6050 initialization unsuccessful\n");
                }

                _mpu6050.setFullScaleGyroRange(GRANGE);

                _mpu6050.setFullScaleAccelRange(ARANGE);
            }

            const auto gyroRangeDps() -> int16_t
            {
                static constexpr int16_t granges[4] = {250, 500, 100, 2000};

                return granges[GRANGE];
            }

            const auto accelRangeGs() -> int16_t
            {
                static constexpr int16_t aranges[4] = {2, 4, 8, 16};

                return aranges[ARANGE];
            }

            auto read() -> ImuRaw
            {
                int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

                _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                return ImuRaw(Vec3Raw(gx, gy, gz), Vec3Raw(ax, ay, az));
            }

        private:

            MPU6050 _mpu6050;
    };
}
