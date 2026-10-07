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
#include <firmware/debugger.hpp>
#include <firmware/zranger/sensor.hpp>

#include "error.hpp"

static Adafruit_VL53L1X vl53l1x_;

namespace hf {

    void ZRanger::Begin()
    {
        Wire1.begin();
        Wire1.setClock(400000);
        delay(100);

        if (!vl53l1x_.begin(0x29, &Wire1)) {
            Error::ReportForever("Unable to initialize VL53L1X");
        }

        if (!vl53l1x_.startRanging()) {
            Error::ReportForever("VL53L1X failed to start ranging");
        }

        // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms
        vl53l1x_.setTimingBudget(50);

    }

    auto ZRanger::Read() -> float
    {
        static float _distance;

        if (vl53l1x_.dataReady())  {

            _distance = vl53l1x_.distance();

            // Prepare for another reading
            vl53l1x_.clearInterrupt();
        }

        return _distance;
    }

}
