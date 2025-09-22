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

#include <oneshot125.hpp>

#include <motors.hpp>

static const std::vector<uint8_t> PINS = {PC11, PC12, PB8, PB5};

static auto motors = OneShot125(PINS);

void Motors::device_init()
{
    motors.arm();
}

void Motors::device_setRatio(uint32_t id, uint16_t ratio)
{
    const uint8_t pulse_width = 125 * ((ratio / 65536.f) + 1);

    motors.set(id, pulse_width);
}

void Motors::device_run()
{
    motors.run();
}


