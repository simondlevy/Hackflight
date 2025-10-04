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

#include <tasks/core.hpp>

static const std::vector<uint8_t> MOTOR_PINS = {14, 15, 26, 27};

static auto motors = OneShot125(MOTOR_PINS);

void CoreTask::motors_init()
{
    motors.arm();
}

void CoreTask::motors_setSpeed(uint32_t id, float speed)
{
    const uint8_t pulse_width = 125 * (speed + 1);

    static uint8_t _m1, _m2, _m3;
    if (id == 0) _m1 = pulse_width;
    if (id == 1) _m2 = pulse_width;
    if (id == 2) _m3 = pulse_width;

    if (id == 3) DebugTask::setMessage(_debugTask,
            "m1=%03d m2=%03d m3=%03d m4=%03d",
            _m1, _m2, _m3, pulse_width);

    motors.set(id, pulse_width);
}

void CoreTask::motors_run()
{
    motors.run();
}
