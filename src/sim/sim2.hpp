/*
   Webots-based flight simulator support for Hackflight using custom physics

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


#pragma once

// C
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

// C++
#include <map>
#include <string>

// Hackflight
#include <hackflight.hpp>

// Webots
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

namespace hf {

    class Simulator {

        public:

            void init() 
            {
                wb_robot_init();

                _camera = wb_robot_get_device("camera");

                _timestep = wb_robot_get_basic_time_step();

                wb_camera_enable(_camera, _timestep * 2);

                _emitter = wb_robot_get_device("emitter");

                if (!_emitter) {
                    printf("!!! joystick :: reset :: emitter is not available.\n");
                }

                animateMotor("motor1", -1);
                animateMotor("motor2", +1);
                animateMotor("motor3", +1);
                animateMotor("motor4", -1);
            }

            bool step()
            {
                if (wb_robot_step(_timestep) == -1) {
                    return false;
                }

                // Send joystick value.
                if (_emitter) {

                    double command[3] = {0.0, 0.0, 0.0};

                    if (command[0] || command[1] || command[2]) {
                        printf("command = ( %g , %g , %g )\n",
                                command[0], command[1], command[2]);
                    }
                    wb_emitter_send(_emitter, command, sizeof(command));
                }

                return true;
            }

            void close()
            {
                wb_robot_cleanup();
            }

        private:

            WbDeviceTag _camera;

            WbDeviceTag _emitter;

            double _timestep;

            static void animateMotor(const char * name, const float direction)
            {
                auto motor = wb_robot_get_device(name);

                wb_motor_set_position(motor, INFINITY);

                wb_motor_set_velocity(motor, direction * 60);
            }

    };

}
