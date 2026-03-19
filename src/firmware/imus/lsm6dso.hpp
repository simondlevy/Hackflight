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

#include <LSM6DSOSensor.h>

#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>

namespace hf {

    class IMU {

        private:

            static constexpr int16_t GRANGE = 2000;
            static constexpr int16_t ARANGE = 16; 

        public:

            void begin()
            {
                Wire.begin();

                Wire.setClock(1000000); 

                if (
                        bad(_lsm6dso.begin())  ||
                        bad(_lsm6dso.Enable_G())  ||
                        bad(_lsm6dso.Enable_X())  ||
                        bad(_lsm6dso.Set_X_FS(ARANGE)) ||
                        bad(_lsm6dso.Set_G_FS(GRANGE)))
                {
                    Debugger::reportForever(
                            "LSM6DSO initialization unsuccessful\n");
                }
            }

            auto gyroRangeDps() -> int16_t
            {
                return (int16_t)GRANGE;
            }

            auto accelRangeGs() -> int16_t
            {
                return (int16_t)ARANGE;
            }

            auto read() -> ImuRaw
            {
                int16_t gyro[3] = {};
                _lsm6dso.Get_G_AxesRaw(gyro);

                int16_t accel[3] = {};
                _lsm6dso.Get_X_AxesRaw(accel);

                return ImuRaw(
                        hf::Vec3Raw(gyro[0], gyro[1], gyro[2]),
                        hf::Vec3Raw(accel[0], accel[1], accel[2]));
            }

        private:

            static bool bad(const LSM6DSOStatusTypeDef status)
            {
                return status != LSM6DSO_OK;
            }

            LSM6DSOSensor _lsm6dso = LSM6DSOSensor(&Wire);
    };
}
