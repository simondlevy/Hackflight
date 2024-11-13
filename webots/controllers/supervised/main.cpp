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

// C
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

// Hackflight
#include <sim/dynamics.hpp>

// Webots
#include <webots/robot.h>
#include <webots/supervisor.h>

static const hf::Dynamics::vehicle_params_t vparams = {

    3.264065e-5, // b thrust coefficient [F=b*w^2]
    0.03,        // l arm length [m]

    2.e-06,      // drag coefficient [T=d*w^2]
    0.05,        // m mass [kg]
    2,           // Ix [kg*m^2] 
    2,           // Iy [kg*m^2] 
    3,           // Iz [kg*m^2] 
    3.8e-03      // Jr prop inertial [kg*m^2] 
};

typedef struct {

    hf::Dynamics * dynamics;
    float z;
    bool running;

} thread_data_t;


static void * thread_fun(void *ptr)
{
    auto thread_data = (thread_data_t *)ptr;

    auto dynamics = thread_data->dynamics;

    const float MOTOR = 55.385; // rad/sec

    const float motors[4] = { MOTOR, MOTOR, MOTOR, MOTOR };

    while (thread_data->running) {

        dynamics->update(motors);

        const auto state = dynamics->getState();

        thread_data->z = state.z;
    }

    return  ptr;
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;


    // Create quadcopter dynamics model
    auto dynamics = hf::Dynamics(vparams); 

    // Set up initial conditions
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    wb_robot_init();

    const auto wb_timestep = wb_robot_get_basic_time_step();

    auto copter_node = wb_supervisor_node_get_from_def("ROBOT");

    auto translation_field =
        wb_supervisor_node_get_field(copter_node, "translation");

    // Start the dynamics thread
    thread_data_t thread_data = {
        &dynamics,
        0,
        true
    };

    pthread_t thread = {}; 

    pthread_create(
            &thread, NULL, *thread_fun, (void *)&thread_data);

    while (true) {

        if (wb_robot_step((int)wb_timestep) == -1) {

            thread_data.running = false;
            break;
        } 

        (void)translation_field;

        printf("%f\n", thread_data.z);
    }

    return 0;
}
