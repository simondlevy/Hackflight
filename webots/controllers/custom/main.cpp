/* 
   C++ flight simulator support for Hackflight with custom physics plugin
   
   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// C++
#include <map>
#include <string>

// Hackflight
#include <hackflight.hpp>

// Webots
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

static constexpr float THROTTLE_SCALE = 0.5; // m/s

static constexpr float YAW_SCALE = 160; // deg/s

static void animateMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);

    wb_motor_set_velocity(motor, direction * 60);
}

static hf::siminfo_t getSimInfoFromKeyboard()
{
    static bool _spacebar_was_hit;

    hf::siminfo_t siminfo = {};

    siminfo.is_springy = true;

    switch (wb_keyboard_get_key()) {

        case WB_KEYBOARD_UP:
            siminfo.demands.pitch = +1.0;
            break;

        case WB_KEYBOARD_DOWN:
            siminfo.demands.pitch = -1.0;
            break;

        case WB_KEYBOARD_RIGHT:
            siminfo.demands.roll = +1.0;
            break;

        case WB_KEYBOARD_LEFT:
            siminfo.demands.roll = -1.0;
            break;

        case 'Q':
            siminfo.demands.yaw = -YAW_SCALE;
            break;

        case 'E':
            siminfo.demands.yaw = +YAW_SCALE;
            break;

        case 'W':
            siminfo.demands.thrust = +THROTTLE_SCALE;
            break;

        case 'S':
            siminfo.demands.thrust = -THROTTLE_SCALE;
            break;

        case 32:
            _spacebar_was_hit = true;
            break;
    }

    siminfo.requested_takeoff = _spacebar_was_hit;

    return siminfo;
}


int main() 
{
    wb_robot_init();

    auto timestep = wb_robot_get_basic_time_step();

    auto emitter = wb_robot_get_device("emitter");

    wb_keyboard_enable(timestep);

    animateMotor("motor1", -1);
    animateMotor("motor2", +1);
    animateMotor("motor3", +1);
    animateMotor("motor4", -1);

    while (true) {

        if (wb_robot_step(timestep) == -1) {
            break;
        }

        const auto siminfo = getSimInfoFromKeyboard();

        wb_emitter_send(emitter, &siminfo, sizeof(siminfo));
    }

    wb_robot_cleanup();

    return 0;
}
