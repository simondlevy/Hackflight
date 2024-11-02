/* 
   C++ flight simulator playback program 
   
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
#include <string.h>

#include <webots/motor.h>
#include <webots/robot.h>

static const char * LOGFILE = "pose.csv";

static WbDeviceTag makeMotor(const char * name)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);

    return motor;
}

static float parse(char ** pch)
{
    const auto val = atof(*pch);
    *pch = strtok(NULL, ",");
    return val;
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    wb_robot_init();

    auto motor1 = makeMotor("motor1");
    auto motor2 = makeMotor("motor2");
    auto motor3 = makeMotor("motor3");
    auto motor4 = makeMotor("motor4");

    const auto timestep = wb_robot_get_basic_time_step();

    auto fp = fopen(LOGFILE, "r");

    while (true) {

        char line[100] = {};

        if (fgets(line, 100, fp) == NULL) {
            break;
        }

        line[strlen(line) - 1] = 0;

        auto pch = strtok (line, ",");

        auto x = parse(&pch);
        auto y = parse(&pch);
        auto z = parse(&pch);
        auto phi = parse(&pch);
        auto theta = parse(&pch);
        auto psi = parse(&pch);
        auto m1 = parse(&pch);
        auto m2 = parse(&pch);
        auto m3 = parse(&pch);
        auto m4 = parse(&pch);

        // Negate expected direction to accommodate Webots
        // counterclockwise positive
        wb_motor_set_velocity(motor1, -m1);
        wb_motor_set_velocity(motor2, +m2);
        wb_motor_set_velocity(motor3, +m3);
        wb_motor_set_velocity(motor4, -m4);

        if (wb_robot_step((int)timestep) == -1) {
            break;
        } 
    }

    return 0;
}
