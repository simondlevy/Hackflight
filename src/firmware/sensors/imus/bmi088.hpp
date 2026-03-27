/*
   Hackflight for Teensy 4.0 with ELRs receiver

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

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <BMI088.h>

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>

namespace hf {

    class IMU {

        private:

            static constexpr Bmi088Gyro::Range GRANGE = Bmi088Gyro::RANGE_2000DPS;
            static constexpr Bmi088Accel::Range ARANGE = Bmi088Accel::RANGE_24G;

        public:

            void begin()
            {
                if (!(
                            okay(gyro.begin()) ||

                            okay(accel.begin()) ||

                            okay(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) ||

                            okay(gyro.setRange(GRANGE)) ||

                            okay(gyro.pinModeInt3(
                                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) ||

                            okay(gyro.mapDrdyInt3(true)) ||

                            okay(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) ||

                            okay(accel.setRange(ARANGE)))) {

                    Debugger::reportForever("BMI088 initialization unsuccessful");
                }
            }

            const auto gyroRangeDps() -> int16_t
            {
                static constexpr int16_t granges[5] = {2000, 1000, 500, 250, 125};

                return granges[GRANGE];
            }

            const auto accelRangeGs() -> int16_t
            {
                static constexpr int16_t aranges[4] = {3, 6, 12, 24};

                return aranges[ARANGE];
            }

            auto read() -> ImuRaw
            {
                gyro.readSensor();

                accel.readSensor();

                return ImuRaw(
                        Vec3Raw(
                            gyro.getGyroX_raw(),
                            gyro.getGyroY_raw(),
                            gyro.getGyroZ_raw()
                            ),
                        Vec3Raw(
                            accel.getAccelX_raw(),
                            accel.getAccelY_raw(),
                            accel.getAccelZ_raw()
                            ));
            }

        private:

            Bmi088Accel accel = Bmi088Accel(Wire, 0x19);
            Bmi088Gyro gyro = Bmi088Gyro(Wire, 0x69);

            static bool okay(const int status)
            {
                return status >= 0;
            }

    };
}
