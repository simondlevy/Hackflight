/* 
   C++ flight simulator support for Hackflight with custom physics plugin

   Copyright (C) 2025 Simon D. Levy

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

// C/C++
#include <map>
#include <string>
using namespace std;

// Hackflight
#include <datatypes.h>
#include <simulator/simulator.hpp>

// Webots
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

class Support {

    private:

        float _zdist;

        double _start_x, _start_y, _start_z;

        flightMode_t _flightMode;

        typedef struct {

            int8_t throttle;
            int8_t roll;
            int8_t pitch;
            int8_t yaw;

        } joystick_t;

        std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

            {"Logitech Gamepad F310", joystick_t {-2,  4, -5, 1 } },

            {"Microsoft X-Box 360 pad", joystick_t {-2,  4, -5, 1 } }
        };

        typedef bool (*flightModeFun_t)(const flightMode_t);

        void climb(const float rate)
        {
            const float time_curr = platform_get_time();

            static float _time_prev;

            const float dt = _time_prev > 0 ? time_curr - _time_prev : 0;

            _time_prev = time_curr;

            _zdist = std::min(std::max(
                        _zdist + rate * Simulator::ZDIST_HOVER_INC_MPS * dt,
                        Simulator::ZDIST_HOVER_MIN_M), Simulator::ZDIST_HOVER_MAX_M);
        }

        void getSetpointFromKey(const int key, Simulator::info_t & siminfo)
        {
            switch (key) {

                case WB_KEYBOARD_UP:
                    siminfo.setpoint.pitch = +1.0;
                    break;

                case WB_KEYBOARD_DOWN:
                    siminfo.setpoint.pitch = -1.0;
                    break;

                case WB_KEYBOARD_RIGHT:
                    siminfo.setpoint.roll = +1.0;
                    break;

                case WB_KEYBOARD_LEFT:
                    siminfo.setpoint.roll = -1.0;
                    break;

                case 'Q':
                    siminfo.setpoint.yaw = -0.5;
                    break;

                case 'E':
                    siminfo.setpoint.yaw = +0.5;
                    break;

                case 'W':
                    climb(+1);
                    break;

                case 'S':
                    climb(-1);
                    break;
            }
        }

        void switchMode(Simulator::toggle_e toggle)
        {
            switch (_flightMode) {

                case MODE_IDLE:
                    _flightMode = toggle == Simulator::TOGGLE_HOVER ? MODE_HOVERING : _flightMode;
                    break;

                case MODE_HOVERING:
                    _flightMode = 
                        toggle == Simulator::TOGGLE_HOVER ?  MODE_LANDING :
                        toggle == Simulator::TOGGLE_AUTO ?  MODE_AUTONOMOUS :
                        _flightMode;
                    break;

                case MODE_AUTONOMOUS:
                    _flightMode = 
                        toggle == Simulator::TOGGLE_AUTO ?  MODE_HOVERING :
                    _flightMode;
                    break;

                default:
                    break;
            }
        }

        void checkKeyboardToggle(
                const int key,
                const int target,
                const Simulator::toggle_e toggle,
                bool & key_was_down)
        {
            if (key == target) {
                if (!key_was_down) {
                    key_was_down = true;
                    switchMode(toggle);
                }
            }
        }

        void checkButtonToggle(
                const int button,
                const int target,
                const Simulator::toggle_e toggle,
                bool & button_was_down)
        {
            if (button == target) {
                button_was_down = true;
            }
            else {
                if (button_was_down) {
                    switchMode(toggle);
                }
                button_was_down = false;
            }
        }

        static float normalizeJoystickAxis(const int32_t rawval)
        {
            return 2.0f * rawval / UINT16_MAX; 
        }

        int32_t readJoystickRaw(const int8_t index)
        {
            const auto axis = abs(index) - 1;
            const auto sign = index < 0 ? -1 : +1;
            return sign * platform_joystick_get_axis_value(axis);
        }

        float readJoystickAxis(const int8_t index)
        {
            return normalizeJoystickAxis(readJoystickRaw(index));
        }


        Simulator::joystickStatus_e getJoystickStatus(void)
        {
            auto mode = Simulator::JOYSTICK_RECOGNIZED;

            auto joyname = platform_joystick_get_model();

            // No joystick
            if (joyname == NULL) {

                static bool _didWarn;

                if (!_didWarn) {
                    puts("Using keyboard instead:\n");
                    puts("- Use Enter to take off and land\n");
                    puts("- Use W and S to go up and down\n");
                    puts("- Use arrow keys to move horizontally\n");
                    puts("- Use Q and E to change heading\n");
                }

                _didWarn = true;

                mode = Simulator::JOYSTICK_NONE;
            }

            // Joystick unrecognized
            else if (JOYSTICK_AXIS_MAP.count(joyname) == 0) {

                mode = Simulator::JOYSTICK_UNRECOGNIZED;
            }

            return mode;
        }

        void reportJoystick(void)
        {
            printf("Unrecognized joystick '%s' with axes ",
                    platform_joystick_get_model()); 

            for (uint8_t k=0; k<platform_joystick_get_number_of_axes(); ++k) {

                printf("%2d=%+6d |", k+1, platform_joystick_get_axis_value(k));
            }
        }

        void sendSimInfo(Simulator::info_t & siminfo)
        {
            if (_start_x == 0) {
                platform_get_vehicle_location(_start_x, _start_y, _start_z);
            }

            siminfo.start_x = _start_x;
            siminfo.start_y = _start_y;
            siminfo.start_z = _start_z;

            siminfo.setpoint.thrust = _zdist;
            siminfo.framerate = platform_get_framerate();
            platform_send_siminfo(siminfo);
        }

        void getSimInfoFromJoystick(
                Simulator::info_t & siminfo,
                flightModeFun_t flight_mode_check)
        {
            static bool _hover_button_was_down;
            static bool _auto_button_was_down;

            auto axes = platform_joystick_get_info();

            const auto button = platform_joystick_get_pressed_button();

            checkButtonToggle(button, 5, Simulator::TOGGLE_HOVER, _hover_button_was_down);

            checkButtonToggle(button, 4, Simulator::TOGGLE_AUTO, _auto_button_was_down);

            siminfo.flightMode = _flightMode;

            if (flight_mode_check(siminfo.flightMode)) {

                siminfo.setpoint.pitch = readJoystickAxis(axes.pitch);
                siminfo.setpoint.roll = readJoystickAxis(axes.roll);
                siminfo.setpoint.yaw = readJoystickAxis(axes.yaw);

                climb(readJoystickAxis(axes.throttle));
            }
        }

        void getSimInfoFromKeyboard(
                Simulator::info_t & siminfo,
                flightModeFun_t flight_mode_check)
        {
            static bool _enter_was_down;
            static bool _spacebar_was_down;

            const int key = platform_keyboard_get_key();

            if (key == -1 ) {
                _enter_was_down = false;
                _spacebar_was_down = false;
            }

            checkKeyboardToggle(key, 4, Simulator::TOGGLE_HOVER, _enter_was_down);

            checkKeyboardToggle(key, 32, Simulator::TOGGLE_AUTO, _spacebar_was_down);

            if (flight_mode_check(_flightMode)) {

                getSetpointFromKey(key, siminfo);
            }

            siminfo.flightMode = _flightMode;
        }

    public:

        bool beginStep(flightModeFun_t flight_mode_check,
                Simulator::info_t & siminfo)
        {
            if (!platform_step()) {
                return false;
            }

            switch (getJoystickStatus()) {

                case Simulator::JOYSTICK_RECOGNIZED:
                    getSimInfoFromJoystick(siminfo, flight_mode_check);
                    break;

                case Simulator::JOYSTICK_UNRECOGNIZED:
                    reportJoystick();
                    // fall thru

                default:
                    getSimInfoFromKeyboard(siminfo, flight_mode_check);
            }

            return true;
        }

        void endStep(Simulator::info_t &siminfo)
        {
            // On descent, switch mode to idle when close enough to ground
            double x=0, y=0, z=0;
            platform_get_vehicle_location(x, y, z);
            if (_flightMode == MODE_LANDING && (z-_start_z )< Simulator::ZDIST_LANDING_MAX_M) {
                _flightMode = MODE_IDLE;
            }

            sendSimInfo(siminfo);
        }

        flightMode_t getFlightMode()
        {
            return _flightMode;
        }

        void begin()
        {
            _flightMode = MODE_IDLE;

            _zdist = Simulator::ZDIST_HOVER_INIT_M;

            platform_init();
        }

        int end()
        {
            platform_cleanup();
            return 0;
        }

        void readRanger(int16_t * distance_mm) 
        {
            platform_read_rangefinder(distance_mm);
        }

        /////////////////////////////////////////////////////////////////////

    private:

        WbDeviceTag _emitter;
        WbDeviceTag _gps;
        WbDeviceTag _ranger;

        double _timestep;

        static void startMotor(const char * name, const float direction)
        {
            auto motor = wb_robot_get_device(name);
            wb_motor_set_position(motor, INFINITY);
            wb_motor_set_velocity(motor, direction * 60);
        }

        float platform_get_time()
        {
            return wb_robot_get_time();
        }

        void platform_get_vehicle_location(double & x, double & y, double & z)
        {
            const double * xyz = wb_gps_get_values(_gps);

            x = xyz[0];
            y = xyz[1];
            z = xyz[2];
        }

        void platform_send_siminfo(const Simulator::info_t & siminfo)
        {
            wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));
        }

        int platform_joystick_get_axis_value(const uint8_t axis)
        {
            return wb_joystick_get_axis_value(axis);
        }

        int platform_joystick_get_pressed_button()
        {
            return wb_joystick_get_pressed_button();
        }

        joystick_t platform_joystick_get_info() 
        {
            return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
        }

        const char * platform_joystick_get_model()
        {
            return wb_joystick_get_model();
        }

        int platform_joystick_get_number_of_axes()
        {
            return wb_joystick_get_number_of_axes();
        }

        int platform_keyboard_get_key()
        {
            return wb_keyboard_get_key();
        }

        void platform_init()
        {
            wb_robot_init();

            _timestep = wb_robot_get_basic_time_step();

            _emitter = wb_robot_get_device("emitter");

            _gps = wb_robot_get_device("gps");
            wb_gps_enable(_gps, _timestep);

            WbDeviceTag camera = wb_robot_get_device("camera");
            wb_camera_enable(camera, _timestep);

            _ranger = wb_robot_get_device("range-finder");
            wb_range_finder_enable(_ranger, _timestep);

            wb_keyboard_enable(_timestep);

            startMotor("motor1", -1);
            startMotor("motor2", +1);
            startMotor("motor3", +1);
            startMotor("motor4", -1);

            wb_joystick_enable(_timestep);
        }

        void platform_cleanup()
        {
            wb_robot_cleanup();
        }

        bool platform_step()
        {
            return wb_robot_step(_timestep) != -1;
        }

        float platform_get_framerate()
        {
            return 1000 / _timestep;
        }

        void platform_read_rangefinder(int16_t * distance_mm) 
        {
            const float * image = wb_range_finder_get_range_image(_ranger);

            const int width = wb_range_finder_get_width(_ranger);
            const int height = wb_range_finder_get_height(_ranger);

            for (int x=0; x<width; ++x) {

                for (int y=0; y<height; ++y) {

                    const float distance_m =
                        wb_range_finder_image_get_depth(image, width, x, y);

                    distance_mm[y*width+x] = isinf(distance_m) ? -1 :
                        (int16_t)(1000 * distance_m);
                }
            }
        }

 };
