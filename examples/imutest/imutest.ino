/*
   Hackflight main sketch

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

// Hackflight library
#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/sensors/imus/lsm6dso_rot90ccw.hpp>

static hf::IMU _lsm6dso;

void setup()
{
    _lsm6dso.begin();

}

void loop()
{
    static uint32_t _lcount;
    if (_lsm6dso.available()) {
        /*const auto lsm6dso_raw =*/ _lsm6dso.read();
        //hf::Debugger::report(lsm6dso_raw);
        _lcount++;
    }

    static hf::Timer _timer;

    if (_timer.ready(1)) {
        printf("l=%lu\n", _lcount);
        //_mcount = 0;
        _lcount = 0;
    }
}
