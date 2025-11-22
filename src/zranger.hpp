/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Wire.h>
#include <VL53L1X.h>

class ZRanger {

    public:

        static void init(TwoWire * wire, VL53L1X & vl53l1x)
        {
            wire->begin();
            wire->setClock(400000);
            delay(100);

            vl53l1x.setBus(wire);

            if (!vl53l1x.init()) {
                //Debugger::error("ZRanger");
            }

            vl53l1x.setDistanceMode(VL53L1X::Medium);
            vl53l1x.setMeasurementTimingBudget(25000);

            vl53l1x.startContinuous(50);
         }
};
