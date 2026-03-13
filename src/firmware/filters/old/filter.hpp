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

#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/filters/old/three_axis.hpp>

namespace hf {

    class ImuFilter {

        private:

            static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
            static constexpr float GYRO_SCALE_FACTOR = 131;

            static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
            static constexpr float ACCEL_SCALE_FACTOR = 16384;

            static constexpr float B_ACCEL = 0.14;     
            static constexpr float B_GYRO = 0.1;       

            static constexpr float ACCEL_ERROR_X = 0.0;
            static constexpr float ACCEL_ERROR_Y = 0.0;
            static constexpr float ACCEL_ERROR_Z = 0.0;
            static constexpr float GYRO_ERROR_X = 0.0;
            static constexpr float GYRO_ERROR_Y= 0.0;
            static constexpr float GYRO_ERROR_Z = 0.0;

        public:

            auto run(const ImuRaw &imuraw) -> ImuFiltered
            {
                const auto gyro = imuraw.gyro;
                const auto gyroDps = _gyroFilter.run(
                        hf::Vec3(gyro.x, gyro.y, gyro.z),
                        hf::Vec3(GYRO_ERROR_X, GYRO_ERROR_Y, GYRO_ERROR_Z),
                        GYRO_SCALE_FACTOR,  B_GYRO);


                const auto accel = imuraw.accel;
                const auto accelGs = _accelFilter.run(
                        hf::Vec3(accel.x, accel.y, accel.z),
                        hf::Vec3(ACCEL_ERROR_X, ACCEL_ERROR_Y, ACCEL_ERROR_Z),
                        ACCEL_SCALE_FACTOR,  B_ACCEL);

                return ImuFiltered(gyroDps, accelGs);
            }

        private:

            ThreeAxisFilter _gyroFilter;
            ThreeAxisFilter _accelFilter;
    };

}
