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
#include <simulator/common.h>

class SimOuterLoop {

    private:

        static constexpr float ZDIST_HOVER_INIT_M = 0.4;
        static constexpr float ZDIST_HOVER_MAX_M = 1.0;
        static constexpr float ZDIST_HOVER_MIN_M = 0.2;
        static constexpr float ZDIST_LANDING_MAX_M = 0.01;
        static constexpr float ZDIST_HOVER_INC_MPS = 0.2;

        typedef enum {

            JOYSTICK_NONE,
            JOYSTICK_UNRECOGNIZED,
            JOYSTICK_RECOGNIZED

        } joystickStatus_e;

        typedef enum {

            TOGGLE_HOVER,
            TOGGLE_AUTO

        } toggle_e;

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
                        _zdist + rate * ZDIST_HOVER_INC_MPS * dt,
                        ZDIST_HOVER_MIN_M), ZDIST_HOVER_MAX_M);
        }

        void getSetpointFromKey(const int key, siminfo_t & siminfo)
        {
            if (key ==platform_keyboard_up()) {
                siminfo.setpoint.pitch = +1.0;
            }

            else if (key ==platform_keyboard_down()) {
                siminfo.setpoint.pitch = -1.0;
            }

            else if (key ==platform_keyboard_right()) {
                siminfo.setpoint.roll = +1.0;
            }

            else if (key ==platform_keyboard_left()) {
                siminfo.setpoint.roll = -1.0;
            }

            else if (key =='Q') {
                siminfo.setpoint.yaw = -0.5;
            }

            else if (key =='E') {
                siminfo.setpoint.yaw = +0.5;
            }

            else if (key =='W') {
                climb(+1);
            }

            else if (key =='S') {
                climb(-1);
            }
        }

        void switchMode(toggle_e toggle)
        {
            switch (_flightMode) {

                case MODE_IDLE:
                    _flightMode = toggle == TOGGLE_HOVER ? MODE_HOVERING : _flightMode;
                    break;

                case MODE_HOVERING:
                    _flightMode = 
                        toggle == TOGGLE_HOVER ?  MODE_LANDING :
                        toggle == TOGGLE_AUTO ?  MODE_AUTONOMOUS :
                        _flightMode;
                    break;

                case MODE_AUTONOMOUS:
                    _flightMode = 
                        toggle == TOGGLE_AUTO ?  MODE_HOVERING :
                        _flightMode;
                    break;

                default:
                    break;
            }
        }

        void checkKeyboardToggle(
                const int key,
                const int target,
                const toggle_e toggle,
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
                const toggle_e toggle,
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


        joystickStatus_e getJoystickStatus(void)
        {
            auto mode = JOYSTICK_RECOGNIZED;

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

                mode = JOYSTICK_NONE;
            }

            // Joystick unrecognized
            else if (JOYSTICK_AXIS_MAP.count(joyname) == 0) {

                mode = JOYSTICK_UNRECOGNIZED;
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

        void sendSimInfo(siminfo_t & siminfo)
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
                siminfo_t & siminfo,
                flightModeFun_t flight_mode_check)
        {
            static bool _hover_button_was_down;
            static bool _auto_button_was_down;

            auto axes = platform_joystick_get_info();

            const auto button = platform_joystick_get_pressed_button();

            checkButtonToggle(button, 5, TOGGLE_HOVER, _hover_button_was_down);

            checkButtonToggle(button, 4, TOGGLE_AUTO, _auto_button_was_down);

            siminfo.flightMode = _flightMode;

            if (flight_mode_check(siminfo.flightMode)) {

                siminfo.setpoint.pitch = readJoystickAxis(axes.pitch);
                siminfo.setpoint.roll = readJoystickAxis(axes.roll);
                siminfo.setpoint.yaw = readJoystickAxis(axes.yaw);

                climb(readJoystickAxis(axes.throttle));
            }
        }

        void getSimInfoFromKeyboard(
                siminfo_t & siminfo,
                flightModeFun_t flight_mode_check)
        {
            static bool _enter_was_down;
            static bool _spacebar_was_down;

            const int key = platform_keyboard_get_key();

            if (key == -1 ) {
                _enter_was_down = false;
                _spacebar_was_down = false;
            }

            checkKeyboardToggle(key, 4, TOGGLE_HOVER, _enter_was_down);

            checkKeyboardToggle(key, 32, TOGGLE_AUTO, _spacebar_was_down);

            if (flight_mode_check(_flightMode)) {

                getSetpointFromKey(key, siminfo);
            }

            siminfo.flightMode = _flightMode;
        }

    public:

        bool beginStep(flightModeFun_t flight_mode_check,
                siminfo_t & siminfo)
        {
            if (!platform_step()) {
                return false;
            }

            switch (getJoystickStatus()) {

                case JOYSTICK_RECOGNIZED:
                    getSimInfoFromJoystick(siminfo, flight_mode_check);
                    break;

                case JOYSTICK_UNRECOGNIZED:
                    reportJoystick();
                    // fall thru

                default:
                    getSimInfoFromKeyboard(siminfo, flight_mode_check);
            }

            return true;
        }

        void endStep(siminfo_t &siminfo)
        {
            // On descent, switch mode to idle when close enough to ground
            double x=0, y=0, z=0;
            platform_get_vehicle_location(x, y, z);
            if (_flightMode == MODE_LANDING && (z-_start_z )< ZDIST_LANDING_MAX_M) {
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

            _zdist = ZDIST_HOVER_INIT_M;

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

    private:

        // Platform-dependent ------------------------------------------------

        void         platform_cleanup();
        float        platform_get_framerate();
        float        platform_get_time();
        void         platform_get_vehicle_location(double & x, double & y, double & z);
        void         platform_init();
        int          platform_joystick_get_axis_value(const uint8_t axis);
        joystick_t   platform_joystick_get_info(); 
        const char * platform_joystick_get_model();
        int          platform_joystick_get_number_of_axes();
        int          platform_joystick_get_pressed_button();
        int          platform_keyboard_get_key();
        int          platform_keyboard_down();
        int          platform_keyboard_left();
        int          platform_keyboard_right();
        int          platform_keyboard_up();
        void         platform_send_siminfo(const siminfo_t & siminfo);
        void         platform_read_rangefinder(int16_t * distance_mm); 
        bool         platform_step();
 };
