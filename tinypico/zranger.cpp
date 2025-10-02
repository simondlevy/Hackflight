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

#include <VL53L1X.h>
#include <tasks/zranger.hpp>

static const uint8_t I2C_SDA = 33;
static const uint8_t I2C_SCL = 32;

static VL53L1X vl53l1x;

bool ZRangerTask::device_init()
{
    Wire1.begin(I2C_SDA, I2C_SCL, 100000);

    vl53l1x.setBus(&Wire1);

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
