/* 
   C++ flight simulator kinematic program 
   
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
#include <pthread.h>
#include <unistd.h>

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <hackflight.hpp>
#include <pids/altitude.hpp>
#include <sim/vehicles/tinyquad.hpp>

static const float DYNAMICS_DT = 1e-5;

static WbDeviceTag makeMotor(const char * name)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);

    return motor;
}

static float deg2rad(const float deg)
{
    return M_PI * deg / 180;
}

static float max3(const float a, const float b, const float c)
{
    return
        a > b && a > c ? a :
        b > a && b > c ? b :
        c;
}

static float sign(const float val)
{
    return val < 0 ? -1 : +1;
}

static float scale(const float angle, const float maxang)
{
    return sign(angle) * sqrt(fabs(angle) / maxang);
}

static void angles_to_rotation(
        const float phi, const float theta, const float psi,
        double rs[4])
{
    const auto phirad = deg2rad(phi);
    const auto therad = deg2rad(theta);
    const auto psirad = deg2rad(psi);

    const auto maxang = max3(fabs(phirad), fabs(therad), fabs(psirad));

    if (maxang == 0) {
        rs[0] = 0;
        rs[1] = 0;
        rs[2] = 1;
        rs[3] = 0;
    }

    else {

        rs[0] = scale(phi, maxang);
        rs[1] = scale(theta, maxang);
        rs[2] = scale(psi, maxang);
        rs[3] = maxang;
    }
}

static void * thread_fun(void *ptr)
{
    auto state = (hf::state_t *)ptr;

    auto dynamics = Dynamics(tinyquad_params, DYNAMICS_DT);

    (void)state;
    (void)dynamics;

    while (true) {

        usleep(DYNAMICS_DT / 1e-6);
    }

    return  ptr;
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    wb_robot_init();

    const auto copter_node = wb_supervisor_node_get_from_def("ROBOT");

    const auto translation_field =
        wb_supervisor_node_get_field(copter_node, "translation");
        
    const auto rotation_field =
        wb_supervisor_node_get_field(copter_node, "rotation");

    const auto timestep = wb_robot_get_basic_time_step();

    auto motor1 = makeMotor("motor1");
    auto motor2 = makeMotor("motor2");
    auto motor3 = makeMotor("motor3");
    auto motor4 = makeMotor("motor4");

    hf::state_t state = {};

    pthread_t thread = {};

    pthread_create(&thread, NULL, *thread_fun, (void *)&state);

    while (true) {

        if (wb_robot_step((int)timestep) == -1) {
            break;
        } 

        float vals[10] = {};

        const double pos[3] = {vals[0], vals[1], vals[2]};
        wb_supervisor_field_set_sf_vec3f(translation_field, pos);

        double rot[4] = {};
        angles_to_rotation(vals[3], vals[4], vals[5], rot);
        wb_supervisor_field_set_sf_rotation(rotation_field, rot);

        // Negate expected direction to accommodate Webots
        // counterclockwise positive
        wb_motor_set_velocity(motor1, -vals[6]);
        wb_motor_set_velocity(motor2, +vals[7]);
        wb_motor_set_velocity(motor3, +vals[8]);
        wb_motor_set_velocity(motor4, -vals[9]);
    }

    pthread_join(thread, NULL);

    return 0;
}
