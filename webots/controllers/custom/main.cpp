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
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

static constexpr float THROTTLE_SCALE = 0.5; // m/s

static constexpr float YAW_SCALE = 160; // deg/s

static bool _requested_takeoff;

static void animateMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);

    wb_motor_set_velocity(motor, direction * 60);
}

static hf::demands_t getDemandsFromKeyboard()
{
    static bool spacebar_was_hit;

    hf::demands_t demands = {};

    switch (wb_keyboard_get_key()) {

        case WB_KEYBOARD_UP:
            demands.pitch = +1.0;
            break;

        case WB_KEYBOARD_DOWN:
            demands.pitch = -1.0;
            break;

        case WB_KEYBOARD_RIGHT:
            demands.roll = +1.0;
            break;

        case WB_KEYBOARD_LEFT:
            demands.roll = -1.0;
            break;

        case 'Q':
            demands.yaw = -YAW_SCALE;
            break;

        case 'E':
            demands.yaw = +YAW_SCALE;
            break;

        case 'W':
            demands.thrust = +THROTTLE_SCALE;
            break;

        case 'S':
            demands.thrust = -THROTTLE_SCALE;
            break;

        case 32:
            spacebar_was_hit = true;
            break;
    }

    _requested_takeoff = spacebar_was_hit;

    return demands;
}


int main() 
{
    wb_robot_init();

    auto camera = wb_robot_get_device("camera");

    auto timestep = wb_robot_get_basic_time_step();

    wb_camera_enable(camera, timestep * 2);

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

        const auto demands = getDemandsFromKeyboard();

        wb_emitter_send(emitter, &demands, sizeof(demands));
    }

    wb_robot_cleanup();

    return 0;
}
