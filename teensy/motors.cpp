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

#include <hackflight.hpp>

static const std::vector<uint8_t> MOTOR_PINS = {3, 4, 5, 6};

static auto motors = OneShot125(MOTOR_PINS);

void Hackflight::motors_init()
{
    motors.arm();
}

void Hackflight::motors_setSpeed(uint32_t id, float speed)
{
    const uint8_t pulse_width = 125 * (speed + 1);

    motors.set(id, pulse_width);
}

void Hackflight::motors_run()
{
    motors.run();
}


