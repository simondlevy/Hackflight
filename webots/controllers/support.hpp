/* 
   C++ flight simulator support for Hackflight with custom physics plugin
   
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

// C++
#include <map>
#include <string>

// Hackflight
#include <hackflight.hpp>

// Webots
#include <webots/emitter.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <webots/distance_sensor.h>


class Simulator {

    public:

        void init()
        {
            _init();

            wb_joystick_enable(_timestep);

        }

        void initKeyboard()
        {
            _init();

            printKeyboardInstructions();
        }

        bool step()
        {
            if (!_step()) {
                return false;
            }

            auto siminfo = getSimInfo();

            return dispatchSimInfo(siminfo);
        }

        bool stepKeyboard()
        {
            if (!_step()) {
                return false;
            }

            auto siminfo = getSimInfoFromKeyboard();

            return dispatchSimInfo(siminfo);
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

        WbDeviceTag _emitter;

        WbDeviceTag _rangefinder;

        double _timestep;

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

        static void printKeyboardInstructions()
        {
            puts("- Use spacebar to take off\n");
            puts("- Use W and S to go up and down\n");
            puts("- Use arrow keys to move horizontally\n");
            puts("- Use Q and E to change heading\n");
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

        bool _requested_takeoff;

        void _init()
        {
            wb_robot_init();

            _timestep = wb_robot_get_basic_time_step();

            _emitter = wb_robot_get_device("emitter");

            _rangefinder = wb_robot_get_device("rangefinder");

            wb_distance_sensor_enable(_rangefinder, _timestep);

            wb_keyboard_enable(_timestep);

            animateMotor("motor1", -1);
            animateMotor("motor2", +1);
            animateMotor("motor3", +1);
            animateMotor("motor4", -1);
        }

        bool _step()
        {
            const auto result = wb_robot_step(_timestep) != -1;

            printf("h=%3.3f\n", wb_distance_sensor_get_value(_rangefinder));

            return result;
        }

        bool dispatchSimInfo(hf::siminfo_t & siminfo)
        {
            siminfo.framerate = 1000 / _timestep;

            wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));

            return true;
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

        static void animateMotor(const char * name, const float direction)
        {
            auto motor = wb_robot_get_device(name);

            wb_motor_set_position(motor, INFINITY);

            wb_motor_set_velocity(motor, direction * 60);
        }

        static hf::siminfo_t getSimInfoFromKeyboard()
        {
            static bool _spacebar_was_hit;

            hf::siminfo_t siminfo = {};

            siminfo.is_springy = true;

            switch (wb_keyboard_get_key()) {

                case WB_KEYBOARD_UP:
                    siminfo.demands.pitch = +1.0;
                    break;

                case WB_KEYBOARD_DOWN:
                    siminfo.demands.pitch = -1.0;
                    break;

                case WB_KEYBOARD_RIGHT:
                    siminfo.demands.roll = +1.0;
                    break;

                case WB_KEYBOARD_LEFT:
                    siminfo.demands.roll = -1.0;
                    break;

                case 'Q':
                    siminfo.demands.yaw = -YAW_SCALE;
                    break;

                case 'E':
                    siminfo.demands.yaw = +YAW_SCALE;
                    break;

                case 'W':
                    siminfo.demands.thrust = +THROTTLE_SCALE;
                    break;

                case 'S':
                    siminfo.demands.thrust = -THROTTLE_SCALE;
                    break;

                case 32:
                    _spacebar_was_hit = true;
                    break;
            }

            siminfo.requested_takeoff = _spacebar_was_hit;

            return siminfo;
        }

        hf::siminfo_t getSimInfo()
        {
            hf::siminfo_t siminfo = {};

            auto joystickStatus = haveJoystick();

            if (joystickStatus == JOYSTICK_RECOGNIZED) {

                auto axes = getJoystickInfo();

                siminfo.demands.thrust =
                    normalizeJoystickAxis(readJoystickRaw(axes.throttle));

                // Springy throttle stick; keep in interval [-1,+1]
                if (axes.springy) {

                    siminfo.is_springy = true;

                    static bool button_was_hit;

                    if (wb_joystick_get_pressed_button() == 5) {
                        button_was_hit = true;
                    }

                    siminfo.requested_takeoff = button_was_hit;

                    // Run throttle stick through deadband
                    siminfo.demands.thrust =
                        fabs(siminfo.demands.thrust) < 0.05 ? 0 : siminfo.demands.thrust;
                }

                else {

                    static float throttle_prev;
                    static bool throttle_was_moved;

                    // Handle bogus throttle values on startup
                    if (throttle_prev != siminfo.demands.thrust) {
                        throttle_was_moved = true;
                    }

                    siminfo.requested_takeoff = throttle_was_moved;

                    throttle_prev = siminfo.demands.thrust;
                }

                siminfo.demands.roll = readJoystickAxis(axes.roll);
                siminfo.demands.pitch = readJoystickAxis(axes.pitch); 
                siminfo.demands.yaw = readJoystickAxis(axes.yaw) * YAW_SCALE;
            }

            else if (joystickStatus == JOYSTICK_UNRECOGNIZED) {
                reportJoystick();
            }

            else { 

                siminfo = getSimInfoFromKeyboard();

            }

            return siminfo;
        }
};
