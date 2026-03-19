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

        public:

            IMU(
                    const int grange =  2000,
                    const int arange = 16)
                : _grange(grange), _arange(arange) {}

            void begin()
            {
                Wire.begin();

                // Wire.setClock(1000000); 

                if (
                        bad(_lsm6dso.begin())  ||
                        bad(_lsm6dso.Enable_X())  ||
                        bad(_lsm6dso.Enable_G())  ||
                        bad(_lsm6dso.Set_X_FS(_arange)) ||
                        bad(_lsm6dso.Set_G_FS(_grange)))
                {
                    Debugger::reportForever(
                            "LSM6DSO initialization unsuccessful\n");
                }
            }

            auto gyroRangeDps() -> int16_t
            {
                return (int16_t)_grange;
            }

            auto accelRangeGs() -> int16_t
            {
                return (int16_t)_arange;
            }

            auto read() -> ImuRaw
            {
                int32_t accel[3] = {};
                _lsm6dso.Get_X_Axes(accel);

                int32_t gyro[3] = {};
                _lsm6dso.Get_G_Axes(gyro);

                return ImuRaw(
                        hf::Vec3Raw(gyro[0], gyro[1], gyro[2]),
                        hf::Vec3Raw(accel[0], accel[1], accel[2]));
            }

        private:

            static bool bad(const LSM6DSOStatusTypeDef status)
            {
                return status != LSM6DSO_OK;
            }

            int _grange;
            int _arange;

            LSM6DSOSensor _lsm6dso = LSM6DSOSensor(&Wire);
    };
}
