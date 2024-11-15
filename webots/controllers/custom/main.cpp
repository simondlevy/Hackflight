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

#include <stdio.h>

#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <sim/sim2.hpp>

static void animateMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);

    wb_motor_set_velocity(motor, direction * 60);
}

int main() 
{
    wb_robot_init();

    WbDeviceTag camera = wb_robot_get_device("camera");

    const double timestep = wb_robot_get_basic_time_step();

    wb_camera_enable(camera, timestep * 2);

    // Handle emitter
    const auto gEmitter = wb_robot_get_device("emitter");

    if (!gEmitter) {
        printf("!!! joystick :: reset :: emitter is not available.\n");
    }

    animateMotor("motor1", -1);
    animateMotor("motor2", +1);
    animateMotor("motor3", +1);
    animateMotor("motor4", -1);

    while (wb_robot_step(timestep) != -1) {

        // Send joystick value.
        if (gEmitter) {

            double command[3] = {0.0, 0.0, 0.0};

            if (command[0] || command[1] || command[2]) {
                printf("command = ( %g , %g , %g )\n", command[0], command[1], command[2]);
            }
            wb_emitter_send(gEmitter, command, sizeof(command));
        }
    }

    wb_robot_cleanup();

    return 0;
}
