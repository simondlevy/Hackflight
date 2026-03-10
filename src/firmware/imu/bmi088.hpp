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

#include <Arduino.h>
#include <Wire.h>

#include <BMI088.h>

#include <hackflight.h>
#include <firmware/datatypes.hpp>

namespace hf {

    class IMU {

        private:

            static const int16_t GYRO_SCALE = 2000;

            static const int16_t ACCEL_SCALE = 24;

        public:

            IMU() :
                _accel(Bmi088Accel(Wire, 0x19)),
                _gyro(Bmi088Gyro(Wire, 0x69)) {}

            void begin()
            {
                if (!(
                            okay(_gyro.begin()) &&
                            okay(_accel.begin()) &&
                            okay(_gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&
                            okay(_gyro.setRange(Bmi088Gyro::RANGE_2000DPS)) &&
                            okay(_gyro.pinModeInt3(
                                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&
                            okay(_gyro.mapDrdyInt3(true)) &&
                            okay(_accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&
                            okay(_accel.setRange(Bmi088Accel::RANGE_24G)))) {

                    while (true) {
                        printf("Unable to initialize BMI088\n");
                        delay(500);
                    }
                }
            }            

            auto read() -> ImuRaw
            {
                _gyro.readSensor();

                _accel.readSensor();

                return ImuRaw(
                        _gyro.getGyroX_raw(),
                        _gyro.getGyroY_raw(),
                        _gyro.getGyroZ_raw(),
                        _accel.getAccelX_raw(),
                        _accel.getAccelY_raw(),
                        _accel.getAccelZ_raw());
            }

        private:

            static auto okay(const int status) -> bool
            {
                return status >= 0;
            }

            Bmi088Accel _accel;

            Bmi088Gyro _gyro;
    };
}
