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
#include <VL53L1X.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/zranger/sensor.hpp>
using namespace hf;

static VL53L1X _vl53l1x;

void ZRanger::begin()
{
    Wire1.begin();
    Wire1.setClock(400000);
    delay(100);

    _vl53l1x.setBus(&Wire1);

    if (!_vl53l1x.init()) {
        Debugger::reportForever("Unable to initialize sensor");
    }

    _vl53l1x.setDistanceMode(VL53L1X::Medium);
    _vl53l1x.setMeasurementTimingBudget(25000);

    _vl53l1x.startContinuous(50);
}

auto ZRanger::read() -> float
{
    return _vl53l1x.read();
}
