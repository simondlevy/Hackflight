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
#include <webots/joystick.h>
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

                wb_joystick_enable(_timestep);
                wb_keyboard_enable(_timestep);

                _motor1 = _makeMotor("motor1", +1);
                _motor2 = _makeMotor("motor2", -1);
                _motor3 = _makeMotor("motor3", +1);
                _motor4 = _makeMotor("motor4", -1);
            }

            bool step(
                    demands_t & stickDemands,
                    state_t & vehicleState,
                    bool & hitTakeoffButton,
                    bool & completedTakeoff)
            {
                if (wb_robot_step((int)_timestep) == -1) {
                    return false;
                }

                bool button = false;

                _readSticks(stickDemands, button);

                _getVehicleState(vehicleState);

                static bool _button_was_hit;

                if (button) {
                    _button_was_hit = true;
                }

                hitTakeoffButton = _button_was_hit;
                
                stickDemands.thrust *= THROTTLE_SCALE; 

                static uint32_t _tick;
                
                const double time = _button_was_hit ? _tick++ * _timestep / 1000 : 0;

                completedTakeoff = time > TAKEOFF_TIME;

                return true;
            }

            void setMotors(
                    const float m1, 
                    const float m2, 
                    const float m3, 
                    const float m4)
            {
                wb_motor_set_velocity(_motor1, +m1);
                wb_motor_set_velocity(_motor2, -m2);
                wb_motor_set_velocity(_motor3, +m3);
                wb_motor_set_velocity(_motor4, -m4);
            }

            void close(void)
            {
                wb_robot_cleanup();
            }

        private:

            static constexpr float THROTTLE_SCALE = 0.2;

            static constexpr float THRUST_TAKEOFF = 56;

            static constexpr float THRUST_BASE = 55.385;

            static constexpr float TAKEOFF_TIME = 3;

            typedef enum {

                JOYSTICK_NONE,
                JOYSTICK_UNRECOGNIZED,
                JOYSTICK_RECOGNIZED

            } joystickStatus_e;

            double _timestep;

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

            static bool button_stick(const uint8_t axis)
            {
                return wb_joystick_get_pressed_button() == axis;
            }

            static bool button_stick_0()
            {
                return button_stick(0);
            }

            static bool button_stick_5()
            {
                return button_stick(5);
            }

            static bool button_frsky()
            {
                return readJoystickRaw(5) > 0;
            }

            static bool button_spektrum()
            {
                return readJoystickRaw(5) < 0;
            }

            typedef struct {

                int8_t throttle;
                int8_t roll;
                int8_t pitch;
                int8_t yaw;

                bool (* button_fun)();

            } joystick_t;

            std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

                { "MY-POWER CO.,LTD. 2In1 USB Joystick", // PS3
                    joystick_t {-2,  3, -4, 1, button_stick_5} },
                { "SHANWAN Android Gamepad",             // PS3
                    joystick_t {-2,  3, -4, 1, button_stick_5} },
                { "Logitech Gamepad F310",
                    joystick_t {-2,  4, -5, 1, button_stick_5} },
                { "Logitech Logitech Extreme 3D",
                    joystick_t {-4,  1, -2, 3, button_stick_0}  },
                { "FrSky FrSky Simulator",
                    joystick_t { 1,  2,  3, 4, button_frsky } },
                { "Horizon Hobby SPEKTRUM RECEIVER",
                    joystick_t { 2,  -3,  4, -1, button_spektrum } }
            };

            static float normalizeJoystickAxis(const int32_t rawval)
            {
                return 2.0f * rawval / UINT16_MAX; 
            }

            static int32_t readJoystickRaw(const int8_t index)
            {
                const auto axis = abs(index) - 1;
                const auto sign = index < 0 ? -1 : +1;
                return sign * wb_joystick_get_axis_value(axis);
            }

            static float readJoystickAxis(const int8_t index)
            {
                return normalizeJoystickAxis(readJoystickRaw(index));
            }

            void readJoystick(demands_t & demands, bool & button)
            {
                auto joyname = wb_joystick_get_model();

                auto axes = JOYSTICK_AXIS_MAP[joyname];

                demands.thrust = normalizeJoystickAxis(readJoystickRaw(axes.throttle));

                demands.roll = readJoystickAxis(axes.roll);
                demands.pitch = readJoystickAxis(axes.pitch); 
                demands.yaw = readJoystickAxis(axes.yaw);

                button = axes.button_fun();

                // Run throttle stick through deadband
                demands.thrust = fabs(demands.thrust) < 0.05 ? 0 : demands.thrust;

                // Handle bogus large demands.thrust values on startup
                if (!ready && demands.thrust > -1.0) {
                    ready = true;
                }

                demands.thrust = ready ? demands.thrust : 0;
            }

            static void readKeyboard(demands_t & demands, bool & button)
            {
                switch (wb_keyboard_get_key()) {

                    case WB_KEYBOARD_UP:
                        demands.pitch = +0.5;
                        break;

                    case WB_KEYBOARD_DOWN:
                        demands.pitch = -0.5;
                        break;

                    case WB_KEYBOARD_RIGHT:
                        demands.roll = +0.5;
                        break;

                    case WB_KEYBOARD_LEFT:
                        demands.roll = -0.5;
                        break;

                    case 'Q':
                        demands.yaw = -0.5;
                        break;

                    case 'E':
                        demands.yaw = +0.5;
                        break;

                    case 'W':
                        demands.thrust = +0.5;
                        break;

                    case 'S':
                        demands.thrust = -0.5;
                        break;

                    case 32: // spacebar
                        button = true;
                        break;
                }
            }

            joystickStatus_e haveJoystick(void)
            {
                auto status = JOYSTICK_RECOGNIZED;

                auto joyname = wb_joystick_get_model();

                // No joystick
                if (joyname == NULL) {

                    static bool _didWarn;

                    if (!_didWarn) {
                        puts("Using keyboard instead:\n");
                        puts("- Use W and S to go up and down\n");
                        puts("- Use arrow keys to move in the horizontal plane\n");
                        puts("- Use Q and E to rotate around yaw\n");
                    }

                    _didWarn = true;

                    status = JOYSTICK_NONE;
                }

                // Joystick unrecognized
                else if (JOYSTICK_AXIS_MAP.count(joyname) == 0) {

                    status = JOYSTICK_UNRECOGNIZED;
                }

                return status;
            }

            static void reportJoystick(void)
            {
                printf("Unrecognized joystick '%s' with axes ",
                        wb_joystick_get_model()); 

                for (uint8_t k=0; k<wb_joystick_get_number_of_axes(); ++k) {

                    printf("%2d=%+6d |", k+1, wb_joystick_get_axis_value(k));
                }
            }

            static WbDeviceTag _makeMotor(
                    const char * name, const float direction)
            {
                auto motor = wb_robot_get_device(name);

                wb_motor_set_position(motor, INFINITY);
                wb_motor_set_velocity(motor, direction);

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

            void _readSticks(demands_t & demands, bool & button)
            {
                auto joystickStatus = haveJoystick();

                demands.thrust = 0;
                demands.roll = 0;
                demands.pitch = 0;
                demands.yaw = 0;

                button = false;

                if (joystickStatus == JOYSTICK_RECOGNIZED) {
                    readJoystick(demands, button);
                }

                else if (joystickStatus == JOYSTICK_UNRECOGNIZED) {
                    reportJoystick();
                }

                else {
                    readKeyboard(demands, button);
                }
            }

            void _getVehicleState(state_t & state)
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
    };

}
