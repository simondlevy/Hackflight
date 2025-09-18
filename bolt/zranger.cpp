/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <tasks/zranger.hpp>

#include <VL53L1X.h>

static VL53L1X vl53l1x;

bool ZRangerTask::device_init()
{
    Wire.begin();
    Wire.setClock(400000);
    delay(100);

    if (!vl53l1x.init()) {
        return false;
    }

    vl53l1x.setDistanceMode(VL53L1X::Medium);
    vl53l1x.setMeasurementTimingBudget(25000);
    vl53l1x.startContinuous(50);

    return true;
}


float ZRangerTask::device_read()
{
    return vl53l1x.read();
}
