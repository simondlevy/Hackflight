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
                    const int32_t grange =  2000,
                    const int32_t arange = 16)
                : _grange(grange), _arange(arange) {}

            void begin()
            {
                Wire.begin();

                // Wire.setClock(1000000); 

                if (
                        bad(_lsm6.begin())  ||
                        bad(_lsm6.Enable_X())  ||
                        bad(_lsm6.Enable_G())  ||
                        bad(_lsm6.set_X_FS(_arange)) ||
                        bad(_lsm6.set_G_FS(_grange)))
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
                int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

                _lsm6.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

                return ImuRaw(Vec3Raw(gx, gy, gz), Vec3Raw(ax, ay, az));
            }

        private:

            static bool bad(const LSM6DSOStatusTypeDef status)
            {
                return status != LSM6DSO_OK;
            }

            lsm6dso_fs_g_t _grange;
            lsm6dso_fs_xl_t _arange;

            LSM6DSOSensor _lsm6dso;
    };
}
