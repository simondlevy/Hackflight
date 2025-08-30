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

#include <Arduino.h>

#include <motor_api.h>
#include <oneshot125.hpp>
#include <vector>

static const std::vector<uint8_t> PINS = {2, 23, 14, 9};

static auto motors = OneShot125(PINS);

int motorsGetRatio(uint32_t id)
{
    return 0;
}

void motorsInit(void)
{
    motors.arm(); 
}

void motorsSetRatios(const uint16_t ratios[])
{
    (void)ratios;
}

void  motorsStop()
{
    const uint8_t pulseWidth = 125;

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);
    motors.set(2, pulseWidth);
    motors.set(3, pulseWidth);

    motors.run();
}
