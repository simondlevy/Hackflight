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
#include <webots/motor.h>

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

static const float MOTOR = 55.385; // rad/sec

static const double SPINUP_TIME = 2.5;

typedef struct {

    hf::Dynamics * dynamics;
    float z;
    bool running;

} thread_data_t;


static void * thread_fun(void *ptr)
{
    auto thread_data = (thread_data_t *)ptr;

    auto dynamics = thread_data->dynamics;

    const float motors[4] = { MOTOR, MOTOR, MOTOR, MOTOR };

    while (thread_data->running) {

        dynamics->update(motors);

        const auto state = dynamics->getState();

        thread_data->z = state.z;

        usleep(1);
    }

    return  ptr;
}

static WbDeviceTag make_motor(const char * name)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);

    return motor;
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    // Create quadcopter dynamics model
    auto dynamics = hf::Dynamics(vparams); 

    wb_robot_init();

    const auto timestep = wb_robot_get_basic_time_step();

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

    auto motor1 = make_motor("motor1");
    auto motor2 = make_motor("motor2");
    auto motor3 = make_motor("motor3");
    auto motor4 = make_motor("motor4");

    // Negate expected direction to accommodate Webots
    // counterclockwise positive
    wb_motor_set_velocity(motor1, -MOTOR);
    wb_motor_set_velocity(motor2, +MOTOR);
    wb_motor_set_velocity(motor3, +MOTOR);
    wb_motor_set_velocity(motor4, -MOTOR);

    // Spin up motors in animation
    for (int k=0; k<(int)(SPINUP_TIME*timestep); ++k) {

        const double pos[3] = {0, 0, 0};
        wb_supervisor_field_set_sf_vec3f(translation_field, pos);
    
        wb_robot_step((int)timestep);
    }

    // Set up initial conditions
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    // Start dynamics thread
    pthread_create( &thread, NULL, *thread_fun, (void *)&thread_data);

    // Run to completion
    while (true) {

        if (wb_robot_step((int)timestep) == -1) {
            break;
        }

        const double pos[3] = {0, 0, thread_data.z};
        wb_supervisor_field_set_sf_vec3f(translation_field, pos);

        const auto time_curr = dynamics.getTime();

        const auto state = dynamics.getState();

        if (time_curr > 0) {

            printf("t=%05f  z=%3.3f\n", time_curr, (double)state.z);
        }

    }

    thread_data.running = false;

    pthread_join(thread, NULL);

    return 0;
}
