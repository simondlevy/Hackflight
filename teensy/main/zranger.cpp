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

#include <Wire.h>
#include <VL53L1X.h>

#include <zranger_api.h>

VL53L1X _vl53l1x;

bool zranger_deviceInit()
{
    Wire1.begin();
    Wire1.setClock(400000);

    _vl53l1x.setBus(&Wire1);

    if (!_vl53l1x.init()) {
        return false;
    }

    _vl53l1x.setDistanceMode(VL53L1X::Medium);
    _vl53l1x.setMeasurementTimingBudget(25000); // usec
    _vl53l1x.startContinuous(25); // msec

    return true;
}

float zranger_deviceRead()
{
    return (float)_vl53l1x.read();
}
