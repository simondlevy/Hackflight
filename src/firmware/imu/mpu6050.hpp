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

namespace hf {

    class IMU {

        private:

            static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;

            static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;

            void begin()
            {
                Wire.begin();

                Wire.setClock(1000000); 

                _mpu6050.initialize();

                if (!_mpu6050.testConnection()) {
                    while (true) {
                        printf("MPU6050 initialization unsuccessful\n");
                        delay(500);
                    }
                }

                _mpu6050.setFullScaleGyroRange(GYRO_SCALE);

                _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
            }

            auto getVehicleState(const float dt) -> ImuRaw
            {
                int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

                _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                return ImuRaw(gx, gy, gz, ax, ay, az);
            }

        private:

            MPU6050 _mpu6050;
    };
}
