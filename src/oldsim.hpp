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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <map>
#include <string>

#include <utils.hpp>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

namespace hf {

    class Simulator {

        public:

            void init(void)
            {
                wb_robot_init();

                _timestep = wb_robot_get_basic_time_step();

                _imu = _makeSensor("inertial_unit",
                        _timestep, wb_inertial_unit_enable);
                _gyro = _makeSensor("gyro",
                        _timestep, wb_gyro_enable);
                _gps = _makeSensor("gps",
                        _timestep, wb_gps_enable);
                _camera = _makeSensor("camera",
                        _timestep, wb_camera_enable);

                wb_keyboard_enable(_timestep);

                _motor1 = _makeMotor("motor1");
                _motor2 = _makeMotor("motor2");
                _motor3 = _makeMotor("motor3");
                _motor4 = _makeMotor("motor4");
            }

            bool step(state_t & state, demands_t & demands, bool & requestedTakeoff)
            {
                if (wb_robot_step((int)_timestep) == -1) {
                    return false;
                }

                getDemands(demands, requestedTakeoff);

                getState(state);

                return true;
            }

            float time()
            {
                return _time;
            }

            void setMotors(const quad_motors_t & motors)
            {
                // Negate expected direction to accommodate Webots
                // counterclockwise positive
                wb_motor_set_velocity(_motor1, -motors.m1);
                wb_motor_set_velocity(_motor2, +motors.m2);
                wb_motor_set_velocity(_motor3, +motors.m3);
                wb_motor_set_velocity(_motor4, -motors.m4);
            }

            void close(void)
            {
                wb_robot_cleanup();
            }

        private:

            double _timestep;

            float  _time;

            bool _button_was_hit;

            uint32_t _tick;
                
            WbDeviceTag _motor1;
            WbDeviceTag _motor2;
            WbDeviceTag _motor3;
            WbDeviceTag _motor4;

            WbDeviceTag _camera;
            WbDeviceTag _gps;
            WbDeviceTag _gyro;
            WbDeviceTag _imu;

            // Handles bogus nonzero throttle stick values at startup
            bool ready;

            void getDemands(demands_t & demands, bool & requestedTakeoff)
            {
                bool button = false;

                demands.thrust = 0;
                demands.roll = 0;
                demands.pitch = 0;
                demands.yaw = 0;

                static bool spacebar_was_hit;

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
                        demands.yaw = -1.0;
                        break;

                    case 'E':
                        demands.yaw = +1.0;
                        break;

                    case 'W':
                        demands.thrust = +1.0;
                        break;

                    case 'S':
                        demands.thrust = -1.0;
                        break;

                    case 32: // spacebar
                        spacebar_was_hit = true;
                        button = true;
                        break;
                }

                if (button) {
                    _button_was_hit = true;
                }

                _time = _button_was_hit ? _tick++ * _timestep / 1000 : 0;

                requestedTakeoff = spacebar_was_hit;
            }

            void getState(state_t & state)
            {
                // Track previous time and position for calculating motion
                static float tprev;
                static float xprev;
                static float yprev;
                static float zprev;

                const auto tcurr = wb_robot_get_time();
                const auto dt =  tcurr - tprev;
                tprev = tcurr;

                auto psi = wb_inertial_unit_get_roll_pitch_yaw(_imu)[2];

                state.z = wb_gps_get_values(_gps)[2];

                state.phi = Utils::RAD2DEG*(
                        wb_inertial_unit_get_roll_pitch_yaw(_imu)[0]);

                state.dphi = Utils::RAD2DEG*(
                        wb_gyro_get_values(_gyro)[0]);

                state.theta = Utils::RAD2DEG*(
                        wb_inertial_unit_get_roll_pitch_yaw(_imu)[1]);

                state.dtheta =  Utils::RAD2DEG*(wb_gyro_get_values(_gyro)[1]); 

                state.psi  =  -Utils::RAD2DEG*(psi); 

                state.dpsi =  -Utils::RAD2DEG*(wb_gyro_get_values(_gyro)[2]);

                // Use temporal first difference to get world-cooredinate
                // velocities
                auto x = wb_gps_get_values(_gps)[0];
                auto y = wb_gps_get_values(_gps)[1];
                auto dx = (x - xprev) / dt;
                auto dy = (y - yprev) / dt;
                state.dz = (state.z - zprev) / dt;

                // Rotate X,Y world velocities into body frame to simulate
                // optical-flow sensor
                auto cospsi = cos(psi);
                auto sinpsi = sin(psi);
                state.dx = dx * cospsi + dy * sinpsi;
                state.dy = dx * sinpsi - dy * cospsi;

                // Save past time and position for next time step
                xprev = x;
                yprev = y;
                zprev = state.z;
            }

           static WbDeviceTag _makeMotor(const char * name)
            {
                auto motor = wb_robot_get_device(name);

                wb_motor_set_position(motor, INFINITY);

                return motor;
            }

            static WbDeviceTag _makeSensor(
                    const char * name, 
                    const uint32_t timestep,
                    void (*f)(WbDeviceTag tag, int sampling_period))
            {
                auto sensor = wb_robot_get_device(name);
                f(sensor, timestep);
                return sensor;
            }
    };

}
