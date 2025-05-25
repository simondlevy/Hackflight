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

#include <tasks/imu.hpp>

float ImuTask::gyroRaw2Dps(const int16_t raw)
{
    return (float)raw * 2 * 2000 / 65536.f;
}

float ImuTask::accelRaw2Gs(const int16_t raw)
{
    return (float)raw * 2 * 24 / 65536.f;
}

const float ImuTask::rawGyroVarianceBase()
{
    return 100;
}
