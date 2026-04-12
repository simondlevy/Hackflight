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

// Third-party libraries
#include <Adafruit_VL53L1X.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/zranger/sensor.hpp>
using namespace hf;

static Adafruit_VL53L1X _vl53l1x;

void ZRanger::begin()
{
    Wire1.begin();
    Wire1.setClock(400000);

    if (!_vl53l1x.begin(0x29, &Wire1)) {
        Debugger::reportForever("Unable to initialize sensor");
    }

    if (!_vl53l1x.startRanging()) {
        Debugger::reportForever("Unable to start ranging");
    }

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    _vl53l1x.setTimingBudget(50'000);

    // Clear interrupt to get things started
    _vl53l1x.clearInterrupt();
}

auto ZRanger::read() -> int16_t
{
    return _vl53l1x.distance();
}
