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

// C++
#include <map>
#include <string>

// Hackflight
#include <datatypes.h>

// Webots
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/range_finder.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

class Simulator {

    public:

        void init()
        {
            wb_robot_init();

            _timestep = wb_robot_get_basic_time_step();

            _emitter = wb_robot_get_device("emitter");

            _range_finder = wb_robot_get_device("range-finder");
            wb_range_finder_enable(_range_finder, _timestep);

            _gps = wb_robot_get_device("gps");
            wb_gps_enable(_gps, _timestep);

            WbDeviceTag camera = wb_robot_get_device("camera");
            wb_camera_enable(camera, _timestep);

            wb_keyboard_enable(_timestep);

            animateMotor("motor1", -1);
            animateMotor("motor2", +1);
            animateMotor("motor3", +1);
            animateMotor("motor4", -1);

            wb_joystick_enable(_timestep);

            _zdist = ZDIST_HOVER_INIT_M;
        }

        bool step()
        {
            if (wb_robot_step(_timestep) == -1) {
                return false;
            }

            static flightMode_t _flightMode;

            // runRangefinder();

            siminfo_t siminfo = {};

            switch (getJoystickStatus()) {

                case JOYSTICK_RECOGNIZED:
                    getSimInfoFromJoystick(siminfo, _flightMode);
                    sendSimInfo(siminfo);
                    break;

                case JOYSTICK_UNRECOGNIZED:
                    reportJoystick();
                    break;

                default:
                    getSimInfoFromKeyboard(siminfo, _flightMode);
                    sendSimInfo(siminfo);
            }

            if (_flightMode == MODE_LANDING && wb_gps_get_values(_gps)[2] < ZDIST_LAND_M) {
                _flightMode = MODE_IDLE;
            }

            return true;
        }

        void close()
        {
            wb_robot_cleanup();
        }

        void runRangefinder()
        {
            const int range_finder_width =
                wb_range_finder_get_width(_range_finder);
            const int range_finder_height =
                wb_range_finder_get_height(_range_finder);

            const float * image =
                wb_range_finder_get_range_image(_range_finder);

            for (int i = 0; i < range_finder_width; i++) {
                for (int j = 0; j < range_finder_height; j++) {
                    const float distance = wb_range_finder_image_get_depth(
                            image, range_finder_width, j, i);
                    if (isinf(distance)) {
                        printf(" inf  ");
                    }
                    else {
                        printf("%3.3f ", distance);
                    }
                }
                printf(" \n \n \n");
            }
            printf("-----------------------------------------------\n");
        }

    private:

        static constexpr float ZDIST_HOVER_INIT_M = 0.4;
        static constexpr float ZDIST_HOVER_MAX_M = 1.0;
        static constexpr float ZDIST_HOVER_MIN_M = 0.2;
        static constexpr float ZDIST_HOVER_INC_MPS = 0.2;
        static constexpr float ZDIST_LAND_M = 0.02;

        typedef enum {

            JOYSTICK_NONE,
            JOYSTICK_UNRECOGNIZED,
            JOYSTICK_RECOGNIZED

        } joystickStatus_e;

        typedef enum {

            TOGGLE_HOVER,
            TOGGLE_AUTO

        } toggle_e;

        typedef struct {

            int8_t throttle;
            int8_t roll;
            int8_t pitch;
            int8_t yaw;

        } joystick_t;

        WbDeviceTag _emitter;
        WbDeviceTag _gps;
        WbDeviceTag _range_finder;

        double _timestep;

        float _zdist;

        std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

            {"Logitech Gamepad F310", joystick_t {-2,  4, -5, 1 } },

            {"Microsoft X-Box 360 pad", joystick_t {-2,  4, -5, 1 } }
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

        void sendSimInfo(siminfo_t & siminfo)
        {
            static double _start_x, _start_y, _start_z;

            const double * xyz = wb_gps_get_values(_gps);

            if (_start_x == 0) {
                _start_x = xyz[0];
                _start_y = xyz[1];
                _start_z = xyz[2];
            }

            siminfo.start_x = _start_x;
            siminfo.start_y = _start_y;
            siminfo.start_z = _start_z;

            siminfo.demands.thrust = _zdist;
            siminfo.framerate = 1000 / _timestep;
            wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));
        }

        joystickStatus_e getJoystickStatus(void)
        {
            auto mode = JOYSTICK_RECOGNIZED;

            auto joyname = wb_joystick_get_model();

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

        void getSimInfoFromJoystick(siminfo_t & siminfo, flightMode_t & flightMode)
        {
            static bool _hover_button_was_down;
            static bool _auto_button_was_down;

            auto axes = getJoystickInfo();

            const auto button = wb_joystick_get_pressed_button();

            if (button == 5) {
                _hover_button_was_down = true;
            }
            else {
                if (_hover_button_was_down) {
                    switchMode(flightMode, TOGGLE_HOVER);
                }
                _hover_button_was_down = false;
            }

            if (button == 4) {
                _auto_button_was_down = true;
            }
            else {
                if (_auto_button_was_down) {
                }
                _auto_button_was_down = false;
            }

            siminfo.flightMode = flightMode;

            if (siminfo.flightMode == MODE_HOVERING) {

                siminfo.demands.pitch = readJoystickAxis(axes.pitch);
                siminfo.demands.roll = readJoystickAxis(axes.roll);
                siminfo.demands.yaw = readJoystickAxis(axes.yaw);

                climb(readJoystickAxis(axes.throttle));
            }
        }

        void getSimInfoFromKeyboard(siminfo_t & siminfo, flightMode_t & flightMode)
        {
            static bool _enter_was_down;
            //static bool _spacebar_was_down;

            const int key = wb_keyboard_get_key();

            if (key == -1 ) {
                _enter_was_down = false;
                //_spacebar_was_down = false;
            }

            /*
               else if (key == 32) {
               const bool tapped_spacebar = toggled(_spacebar_was_down);
               if (tapped_spacebar) {
               switchMode2(flightMode);
               }
               }*/

               else if (key == 4) {
                   if (toggled(_enter_was_down)) {
                       switchMode(flightMode, TOGGLE_HOVER);
                   }
               }

               else if (flightMode == MODE_HOVERING) {

                   getSetpointFromKey(key, siminfo);
               }

               siminfo.flightMode = flightMode;
        }

        bool toggled(bool & key_was_down)
        {
            if (!key_was_down) {
                key_was_down = true;
                return true;
            }
            return false;
        }

        void switchMode(flightMode_t & mode, const toggle_e toggle)
        {
            switch (mode) {

                case MODE_IDLE:
                    mode = MODE_HOVERING;
                    break;

                case MODE_HOVERING:
                    mode = MODE_LANDING;
                    break;

                default:
                    break;

            }
        }

        /*
           void switchMode2(flightMode_t & mode)
           {
           mode = (mode == MODE_HOVERING ? MODE_AUTONOMOUS :
           mode == MODE_AUTONOMOUS ? MODE_HOVERING :
           mode);
           }*/

        void getSetpointFromKey(const int key, siminfo_t & siminfo)
        {
            switch (key) {

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
                    siminfo.demands.yaw = -0.5;
                    break;

                case 'E':
                    siminfo.demands.yaw = +0.5;
                    break;

                case 'W':
                    climb(+1);
                    break;

                case 'S':
                    climb(-1);
                    break;
            }

        }

        void climb(const float rate)
        {
            const float time_curr = wb_robot_get_time();

            static float _time_prev;

            const float dt = _time_prev > 0 ? time_curr - _time_prev : 0;

            _time_prev = time_curr;

            _zdist = std::min(std::max(
                        _zdist + rate * ZDIST_HOVER_INC_MPS * dt,
                        ZDIST_HOVER_MIN_M), ZDIST_HOVER_MAX_M);
        }
};

int main() 
{
    Simulator sim = {};

    sim.init();

    while (true) {

        if (!sim.step()) {
            break;
        }
    }

    sim.close();

    return 0;
}
