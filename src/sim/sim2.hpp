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

// C++
#include <map>
#include <string>

// Hackflight
#include <hackflight.hpp>

// Webots
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

namespace hf {

    class Simulator {

        public:

            void init(const bool tryJoystick=true)
            {
                wb_robot_init();

                _camera = wb_robot_get_device("camera");

                _timestep = wb_robot_get_basic_time_step();

                wb_camera_enable(_camera, _timestep * 2);

                _emitter = wb_robot_get_device("emitter");

                if (tryJoystick) {

                    wb_joystick_enable(_timestep);
                }

                else {

                    printKeyboardInstructions();
                }

                wb_keyboard_enable(_timestep);

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

                demands_t demands = {};

                auto joystickStatus = haveJoystick();

                if (joystickStatus == JOYSTICK_RECOGNIZED) {

                    auto axes = getJoystickInfo();

                    demands.thrust =
                        normalizeJoystickAxis(readJoystickRaw(axes.throttle));

                    // Springy throttle stick; keep in interval [-1,+1]
                    if (axes.springy) {

                        static bool button_was_hit;

                        if (wb_joystick_get_pressed_button() == 5) {
                            button_was_hit = true;
                        }

                        _requested_takeoff = button_was_hit;

                        // Run throttle stick through deadband
                        demands.thrust =
                            fabs(demands.thrust) < 0.05 ? 0 : demands.thrust;
                    }

                    else {

                        static float throttle_prev;
                        static bool throttle_was_moved;

                        // Handle bogus throttle values on startup
                        if (throttle_prev != demands.thrust) {
                            throttle_was_moved = true;
                        }

                        _requested_takeoff = throttle_was_moved;

                        throttle_prev = demands.thrust;
                    }

                    demands.roll = readJoystickAxis(axes.roll);
                    demands.pitch = readJoystickAxis(axes.pitch); 
                    demands.yaw = readJoystickAxis(axes.yaw) * YAW_SCALE;
                }

                else if (joystickStatus == JOYSTICK_UNRECOGNIZED) {
                    reportJoystick();
                }

                else { 

                    demands = getDemandsFromKeyboard();

                }

                wb_emitter_send(_emitter, &demands, sizeof(demands_t));

                return true;
            }

            bool isSpringy()
            {
                return haveJoystick() == JOYSTICK_RECOGNIZED ?
                    getJoystickInfo().springy :
                    true; // keyboard
            }

            void close()
            {
                wb_robot_cleanup();
            }

        private:

            static constexpr float THROTTLE_SCALE = 0.5; // m/s

            static constexpr float YAW_SCALE = 160; // deg/s

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

 
            WbDeviceTag _camera;

            WbDeviceTag _emitter;

            double _timestep;

            bool _requested_takeoff;

            static void animateMotor(const char * name, const float direction)
            {
                auto motor = wb_robot_get_device(name);

                wb_motor_set_position(motor, INFINITY);

                wb_motor_set_velocity(motor, direction * 60);
            }

            static void printKeyboardInstructions()
            {
                puts("- Use spacebar to take off\n");
                puts("- Use W and S to go up and down\n");
                puts("- Use arrow keys to move horizontally\n");
                puts("- Use Q and E to change heading\n");
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

            joystick_t getJoystickInfo() 
            {
                return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
            }

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

            static void reportJoystick(void)
            {
                printf("Unrecognized joystick '%s' with axes ",
                        wb_joystick_get_model()); 

                for (uint8_t k=0; k<wb_joystick_get_number_of_axes(); ++k) {

                    printf("%2d=%+6d |", k+1, wb_joystick_get_axis_value(k));
                }
            }

            demands_t getDemandsFromKeyboard()
            {
                static bool spacebar_was_hit;

                demands_t demands = {};

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
                        demands.thrust = +THROTTLE_SCALE;
                        break;

                    case 'S':
                        demands.thrust = -THROTTLE_SCALE;
                        break;

                    case 32:
                        spacebar_was_hit = true;
                        break;
                }

                _requested_takeoff = spacebar_was_hit;

                return demands;
            }

    };

}
