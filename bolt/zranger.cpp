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

bool ZRangerTask::device_init()
{
    /*
    _vl53l1.init(&deckBus, VL53L1_DEFAULT_ADDRESS);

    _vl53l1.begin();

    _vl53l1.setDistanceMode(VL53L1::DISTANCE_MODE_MEDIUM);
    _vl53l1.setTimingBudgetMsec(25);*/

    return true;
}


float ZRangerTask::device_read()
{
    return 0;
}
