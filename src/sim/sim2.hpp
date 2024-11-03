/*
   Webots-based flight simulator support for Hackflight

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


class Simulator {


    public:

        void run()
        {
            wb_robot_init();

            const auto copter_node = wb_supervisor_node_get_from_def("ROBOT");

            const auto translation_field =
                wb_supervisor_node_get_field(copter_node, "translation");

            const auto rotation_field =
                wb_supervisor_node_get_field(copter_node, "rotation");

            const auto timestep = wb_robot_get_basic_time_step();

            auto motor1 = make_motor("motor1");
            auto motor2 = make_motor("motor2");
            auto motor3 = make_motor("motor3");
            auto motor4 = make_motor("motor4");

            // Spin up the motors for a second before starting dynamics
            for (long k=0; k < SPINUP_TIME * timestep; ++k) {

                if (wb_robot_step((int)timestep) == -1) {
                    break;
                } 

                const float motorvals[4] = {
                    MOTOR_MAX, MOTOR_MAX, MOTOR_MAX, MOTOR_MAX
                };

                spin_motors(motor1, motor2, motor3, motor4, motorvals);

                // Keep the vehicle on the ground
                const double pos[3] = {};
                wb_supervisor_field_set_sf_vec3f(translation_field, pos);
            }

            // Start the dynamics thread
            thread_data_t thread_data = {};
            thread_data.running = true;
            pthread_t thread = {};
            pthread_create(&thread, NULL, *thread_fun, (void *)&thread_data);

            while (true) {

                if (wb_robot_step((int)timestep) == -1) {
                    break;
                } 

                auto motorvals = thread_data.motorvals;

                auto posevals = thread_data.posevals;

                const double pos[3] = {posevals[0], posevals[1], posevals[2]};
                wb_supervisor_field_set_sf_vec3f(translation_field, pos);

                double rot[4] = {};
                angles_to_rotation(posevals[3], posevals[4], posevals[5], rot);
                wb_supervisor_field_set_sf_rotation(rotation_field, rot);

                spin_motors(motor1, motor2, motor3, motor4, motorvals);
            }

            thread_data.running = false;

            pthread_join(thread, NULL);
        }

    private:

        static constexpr float INITIAL_ALTITUDE_TARGET = 0.2;

        static constexpr float THRUST_BASE = 55.385;

        static constexpr float DYNAMICS_DT = 1e-4;

        static const uint32_t PID_PERIOD = 1000;

        static constexpr float MOTOR_MAX = 60;

        static constexpr float SPINUP_TIME = 2;

        typedef struct {

            float posevals[6];
            float motorvals[4];
            bool running;

        } thread_data_t;

        static WbDeviceTag make_motor(const char * name)
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

        static float min(const float val, const float maxval)
        {
            return val > maxval ? maxval : val;
        }

        static void * thread_fun(void *ptr)
        {
            auto thread_data = (thread_data_t *)ptr;

            auto dynamics = Dynamics(tinyquad_params, DYNAMICS_DT);

            hf::AltitudePid altitudePid = {};
            hf::state_t state  = {};
            hf::demands_t demands = {};

            float motor = 0;

            for (long k=0; thread_data->running; k++) {

                if (k % PID_PERIOD == 0) {

                    // Reset thrust demand to altitude target
                    demands.thrust = INITIAL_ALTITUDE_TARGET;

                    // Altitude PID controller converts target to thrust demand
                    altitudePid.run(DYNAMICS_DT, state, demands);
                }

                motor = min(demands.thrust + THRUST_BASE, MOTOR_MAX);

                dynamics.setMotors(motor, motor, motor, motor);
                state.z = dynamics.x[Dynamics::STATE_Z];
                state.dz = dynamics.x[Dynamics::STATE_Z_DOT];

                auto posevals = thread_data->posevals;

                posevals[0] = dynamics.x[Dynamics::STATE_X];
                posevals[1] = dynamics.x[Dynamics::STATE_Y];
                posevals[2] = dynamics.x[Dynamics::STATE_Z];
                posevals[3] = dynamics.x[Dynamics::STATE_PHI];
                posevals[4] = dynamics.x[Dynamics::STATE_THETA];
                posevals[5] = dynamics.x[Dynamics::STATE_PSI];

                auto motorvals = thread_data->motorvals;

                motorvals[0] = motor;
                motorvals[1] = motor;
                motorvals[2] = motor;
                motorvals[3] = motor;

                usleep(DYNAMICS_DT / 1e-6);
            }

            return  ptr;
        }

        static void spin_motors(
                WbDeviceTag m1, WbDeviceTag m2, WbDeviceTag m3, WbDeviceTag m4,
                const float motorvals[4])
        {
            // Negate expected direction to accommodate Webots
            // counterclockwise positive
            wb_motor_set_velocity(m1, -motorvals[0]);
            wb_motor_set_velocity(m2, +motorvals[1]);
            wb_motor_set_velocity(m3, +motorvals[2]);
            wb_motor_set_velocity(m4, -motorvals[3]);
        }
};


