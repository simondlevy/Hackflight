/* 
   C++ flight simulator main for Hackflight with custom physics plugin

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
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <map>
#include <string>
using namespace std;


// Hackflight
#include <datatypes.h>
#include <num.hpp>
#include <simulator/common.h>

// Webots
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/range_finder.h>

#include "../common.hpp"

class SimOuterLoop {

    public:

        bool step(
                siminfo_t & siminfo,
                const demands_t * autonomousSetpoint=nullptr)
        {
            if (!platform_step()) {
                return false;
            }

            const bool autonomous = autonomousSetpoint != nullptr;

            switch (getJoystickStatus()) {

                case JOYSTICK_RECOGNIZED:
                    getSimInfoFromJoystick(siminfo, autonomous);
                    break;

                case JOYSTICK_UNRECOGNIZED:
                    reportJoystick();
                    // fall thru

                default:
                    getSimInfoFromKeyboard(siminfo, autonomous);
            }

            if (autonomous) {
                memcpy(&siminfo.setpoint, autonomousSetpoint, sizeof(demands_t));
            }

            // On descent, switch mode to idle when close enough to ground
            Dynamics::pose_t pose = {};
            platform_get_vehicle_pose(pose);
            if (_mode == MODE_LANDING &&
                    (pose.z-_startingPose.z ) < ZDIST_LANDING_MAX_M) {
                _mode = MODE_IDLE;
            }

            sendSimInfo(siminfo);

            return true;
        }

        mode_e getFlightMode()
        {
            return _mode;
        }

        void begin()
        {
            // OOB value to trigger initialization in step()
            _startingPose.x = INFINITY;

            _mode = MODE_IDLE;

            platform_init();
        }

        int end()
        {
            platform_cleanup();
            return 0;
        }

        void readRangefinder(int16_t * distance_mm, int & width, int & height) 
        {
            platform_read_rangefinder(distance_mm, width, height);
        }

    private:

        static constexpr float ZDIST_LANDING_MAX_M = 0.01;

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

        Dynamics::pose_t _startingPose;

        mode_e _mode;

        std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

            {"Logitech Gamepad F310", joystick_t {-2,  4, -5, 1 } },

            {"Microsoft X-Box 360 pad", joystick_t {-2,  4, -5, 1 } }
        };

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
                siminfo.setpoint.thrust = +1.0;
            }

            else if (key =='S') {
                siminfo.setpoint.thrust = -1.0;
            }
        }

        void switchMode(toggle_e toggle)
        {
            switch (_mode) {

                case MODE_IDLE:
                    _mode = toggle == TOGGLE_HOVER ?
                        MODE_HOVERING : _mode;
                    break;

                case MODE_HOVERING:
                    _mode = 
                        toggle == TOGGLE_HOVER ?  MODE_LANDING :
                        toggle == TOGGLE_AUTO ?  MODE_AUTONOMOUS :
                        _mode;
                    break;

                case MODE_AUTONOMOUS:
                    _mode = 
                        toggle == TOGGLE_AUTO ?  MODE_HOVERING :
                        _mode;
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
            // Grab starting pose first time around
            if (_startingPose.x == INFINITY) {
                platform_get_vehicle_pose(_startingPose);
            }

            memcpy(&siminfo.startingPose, &_startingPose, sizeof(Dynamics::pose_t));
            siminfo.framerate = platform_get_framerate();
            platform_send_siminfo(siminfo);
        }

        void getSimInfoFromJoystick(siminfo_t & siminfo, const bool autonomous)
        {
            static bool _hover_button_was_down;
            static bool _auto_button_was_down;

            auto axes = platform_joystick_get_info();

            const auto button = platform_joystick_get_pressed_button();

            checkButtonToggle(button, 5, TOGGLE_HOVER, _hover_button_was_down);

            checkButtonToggle(button, 4, TOGGLE_AUTO, _auto_button_was_down);

            siminfo.mode = _mode;

            if (!autonomous) {

                siminfo.setpoint.pitch = readJoystickAxis(axes.pitch);
                siminfo.setpoint.roll = readJoystickAxis(axes.roll);
                siminfo.setpoint.yaw = readJoystickAxis(axes.yaw);

                siminfo.setpoint.thrust = readJoystickAxis(axes.throttle);
            }
        }

        void getSimInfoFromKeyboard(siminfo_t & siminfo, const bool autonomous)
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

            if (!autonomous) {

                getSetpointFromKey(key, siminfo);
            }

            siminfo.mode = _mode;
        }

        // Platform-dependent ------------------------------------------------

        void         platform_cleanup();
        float        platform_get_framerate();
        float        platform_get_time();
        void         platform_get_vehicle_pose(Dynamics::pose_t & pose);
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
        void         platform_read_rangefinder(int16_t * distance_mm,
                int & width, int & height); 
        bool         platform_step();
};

static WbDeviceTag _emitter;
static WbDeviceTag _gps;
static WbDeviceTag _imu;
static WbDeviceTag _ranger;

static double _timestep;

int main(int argc, char ** argv) 
{
    (void)argc;

    const char * worldname =  argv[1];
    const char * logfilename =  argv[2];

    SimOuterLoop outerLoop = {};

    outerLoop.begin();

    while (true) {

        siminfo_t siminfo = {};
        strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
        strcpy(siminfo.worldname, worldname);
        strcpy(siminfo.logfilename, logfilename);

        if (!outerLoop.step(siminfo)) {
            break;
        }
    }

    return outerLoop.end();
}

float SimOuterLoop::platform_get_time()
{
    return wb_robot_get_time();
}

void SimOuterLoop::platform_get_vehicle_pose(Dynamics::pose_t & pose)
{
    const double * xyz = wb_gps_get_values(_gps);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(_imu);

    pose.x = xyz[0];
    pose.y = -xyz[1]; // negate Y for leftward positive
    pose.z = xyz[2];
    pose.phi = rpy[0];
    pose.theta = rpy[1];
    pose.psi = -rpy[2];
}

void SimOuterLoop::platform_send_siminfo(const siminfo_t & siminfo)
{
    wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));
}

int SimOuterLoop::platform_joystick_get_axis_value(const uint8_t axis)
{
    return wb_joystick_get_axis_value(axis);
}

int SimOuterLoop::platform_joystick_get_pressed_button()
{
    return wb_joystick_get_pressed_button();
}

SimOuterLoop::joystick_t SimOuterLoop::platform_joystick_get_info() 
{
    return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
}

const char * SimOuterLoop::platform_joystick_get_model()
{
    return wb_joystick_get_model();
}

int SimOuterLoop::platform_joystick_get_number_of_axes()
{
    return wb_joystick_get_number_of_axes();
}

int SimOuterLoop::platform_keyboard_get_key()
{
    return wb_keyboard_get_key();
}

void SimOuterLoop::platform_init()
{
    wb_robot_init();

    _timestep = wb_robot_get_basic_time_step();

    _emitter = wb_robot_get_device("emitter");

    _gps = wb_robot_get_device("gps");
    wb_gps_enable(_gps, _timestep);

    _imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(_imu, _timestep);

    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, _timestep);

    _ranger = wb_robot_get_device("range-finder");
    wb_range_finder_enable(_ranger, _timestep);

    wb_keyboard_enable(_timestep);

    startMotors();

    wb_joystick_enable(_timestep);
}

void SimOuterLoop::platform_cleanup()
{
    wb_robot_cleanup();
}

bool SimOuterLoop::platform_step()
{
    return wb_robot_step(_timestep) != -1;
}

float SimOuterLoop::platform_get_framerate()
{
    return 1000 / _timestep;
}

void SimOuterLoop::platform_read_rangefinder(
        int16_t * distance_mm, int & width, int & height) 
{
    const float * image = wb_range_finder_get_range_image(_ranger);

    width = wb_range_finder_get_width(_ranger);
    height = wb_range_finder_get_height(_ranger);

    for (int x=0; x<width; ++x) {

        for (int y=0; y<height; ++y) {

            const float distance_m =
                wb_range_finder_image_get_depth(image, width, x, y);

            distance_mm[y*width+x] = isinf(distance_m) ? -1 :
                (int16_t)(1000 * distance_m);
        }
    }
}
        
int SimOuterLoop::platform_keyboard_down()
{
    return WB_KEYBOARD_DOWN;
}
        
int SimOuterLoop::platform_keyboard_left()
{
    return WB_KEYBOARD_LEFT;
}

int SimOuterLoop::platform_keyboard_right()
{
    return WB_KEYBOARD_RIGHT;
}

int SimOuterLoop::platform_keyboard_up()
{
    return WB_KEYBOARD_UP;
}
