/**
 *
 * Copyright (C) 2025 Simon D. Levy
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

#include <hackflight.h>
#include <tasks/zranger.hpp>

#include <VL53L1X.h>

static VL53L1X vl53l1;

static void error(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void ZRangerTask::hardware_init()
{
    static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;

    if (!vl53l1.init()) {
        error("ZRANGER: Z-down sensor [FAIL]");
    }

    vl53l1.setDistanceMode(VL53L1X::Medium);

    vl53l1.setMeasurementTimingBudget(25000); // usec

    vl53l1.startContinuous(25); // msec
}

float ZRangerTask::hardware_read()
{
    return vl53l1.read();
}
