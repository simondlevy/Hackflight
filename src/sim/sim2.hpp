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

            static constexpr float MOTOR_HOVER = 55.385; // rad/sec

            void init(const bool tryJoystick=true)
            {
                wb_robot_init();

                _wb_timestep = wb_robot_get_basic_time_step();

                if (tryJoystick) {

                    wb_joystick_enable(_wb_timestep);
                }

                else {

                    printKeyboardInstructions();
                }

                wb_keyboard_enable(_wb_timestep);

                _motor1 = make_motor("motor1");
                _motor2 = make_motor("motor2");
                _motor3 = make_motor("motor3");
                _motor4 = make_motor("motor4");
            }

            bool step()
            {
                if (wb_robot_step((int)_wb_timestep) == -1) {

                    return false;
                } 

                demands_t open_loop_demands = {};

                auto joystickStatus = haveJoystick();

                if (joystickStatus == JOYSTICK_RECOGNIZED) {

                    auto axes = getJoystickInfo();

                    open_loop_demands.thrust =
                        normalizeJoystickAxis(readJoystickRaw(axes.throttle));

                    // Springy throttle stick; keep in interval [-1,+1]
                    if (axes.springy) {

                        static bool button_was_hit;

                        if (wb_joystick_get_pressed_button() == 5) {
                            button_was_hit = true;
                        }

                        _requested_takeoff = button_was_hit;

                        // Run throttle stick through deadband
                        open_loop_demands.thrust =
                            fabs(open_loop_demands.thrust) < 0.05 ? 0 : open_loop_demands.thrust;
                    }

                    else {

                        static float throttle_prev;
                        static bool throttle_was_moved;

                        // Handle bogus throttle values on startup
                        if (throttle_prev != open_loop_demands.thrust) {
                            throttle_was_moved = true;
                        }

                        _requested_takeoff = throttle_was_moved;

                        throttle_prev = open_loop_demands.thrust;
                    }

                    open_loop_demands.roll = readJoystickAxis(axes.roll);
                    open_loop_demands.pitch = readJoystickAxis(axes.pitch); 
                    open_loop_demands.yaw = readJoystickAxis(axes.yaw) * YAW_SCALE;
                }

                else if (joystickStatus == JOYSTICK_UNRECOGNIZED) {

                    reportJoystick();
                }

                else { 

                    getDemandsFromKeyboard(open_loop_demands);

                }

                // Negate expected direction to accommodate Webots
                // counterclockwise positive
                //wb_motor_set_velocity(_motor1, -motorvals[0]);
                //wb_motor_set_velocity(_motor2, +motorvals[1]);
                //wb_motor_set_velocity(_motor3, +motorvals[2]);
                //wb_motor_set_velocity(_motor4, -motorvals[3]);

                return true;
            }

            bool isSpringy()
            {
                return haveJoystick() == JOYSTICK_RECOGNIZED ?
                    getJoystickInfo().springy :
                    true; // keyboard
            }
 
        private:

            static constexpr float YAW_SCALE = 160;
            
            typedef enum {

                JOYSTICK_NONE,
                JOYSTICK_UNRECOGNIZED,
                JOYSTICK_RECOGNIZED

            } joystickStatus_e;

            typedef struct {

                int8_t throttle;
                int8_t roll;
                int8_t pitch;
                int8_t yaw;

                bool springy;                

            } joystick_t;

            bool _requested_takeoff;

            double _wb_timestep;

            WbDeviceTag _motor1;
            WbDeviceTag _motor2;
            WbDeviceTag _motor3;
            WbDeviceTag _motor4;

            float _time;

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

                const auto maxang =
                    max3(fabs(phirad), fabs(therad), fabs(psirad));

                if (maxang == 0) {
                    rs[0] = 0;
                    rs[1] = 0;
                    rs[2] = 1;
                    rs[3] = 0;
                }

                else {

                    rs[0] = scale(phi, maxang);
                    rs[1] = scale(theta, maxang);
                    rs[2] = scale(-psi, maxang); // negate for nose-right positive
                    rs[3] = maxang;
                }
            }

            static float min(const float val, const float maxval)
            {
                return val > maxval ? maxval : val;
            }

            std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

                // Springy throttle
                { "MY-POWER CO.,LTD. 2In1 USB Joystick", // PS3
                    joystick_t {-2,  3, -4, 1, true } },
                { "SHANWAN Android Gamepad",             // PS3
                    joystick_t {-2,  3, -4, 1, true } },
                { "Logitech Gamepad F310",
                    joystick_t {-2,  4, -5, 1, true } },

                // Classic throttle
                { "Logitech Logitech Extreme 3D",
                    joystick_t {-4,  1, -2, 3, false}  },
                { "OpenTX FrSky Taranis Joystick",  // USB cable
                    joystick_t { 1,  2,  3, 4, false } },
                { "FrSky FrSky Simulator",          // radio dongle
                    joystick_t { 1,  2,  3, 4, false } },
                { "Horizon Hobby SPEKTRUM RECEIVER",
                    joystick_t { 2,  -3,  4, -1, false } }
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

            joystick_t getJoystickInfo() 
            {
                return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
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
                        printKeyboardInstructions();
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

            static void printKeyboardInstructions()
            {
                puts("- Use spacebar to take off\n");
                puts("- Use W and S to go up and down\n");
                puts("- Use arrow keys to move horizontally\n");
                puts("- Use Q and E to change heading\n");
            }

           void getDemandsFromKeyboard(demands_t & demands)
            {
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
                        demands.yaw = -YAW_SCALE;
                        break;

                    case 'E':
                        demands.yaw = +YAW_SCALE;
                        break;

                    case 'W':
                        demands.thrust = +1.0;
                        break;

                    case 'S':
                        demands.thrust = -1.0;
                        break;

                    case 32:
                        spacebar_was_hit = true;
                        break;
                }

                _requested_takeoff = spacebar_was_hit;
            }

    };

}
