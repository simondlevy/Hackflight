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
#include <simulator/message.h>

// Webots
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/range_finder.h>

// Shared with other Webots controllers
#include "../motors.hpp"

// Support for different joystick models -------------------------------------

typedef struct {

    int8_t throttle;
    int8_t roll;
    int8_t pitch;
    int8_t yaw;

} joystick_t;

static std::map<std::string, joystick_t> JOYSTICK_AXIS_MAP = {

    {"Logitech Gamepad F310", joystick_t {-2,  4, -5, 1 } },

    {"Microsoft X-Box 360 pad", joystick_t {-2,  4, -5, 1 } }
};

// Webots-specific code -------------------------------------------------------

static WbDeviceTag _emitter;
static WbDeviceTag _gps;
static WbDeviceTag _imu;
static WbDeviceTag _ranger;

static double _timestep;

static void platform_init()
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

static bool platform_step()
{
    return wb_robot_step(_timestep) != -1;
}

static const char * platform_joystick_get_model()
{
    return wb_joystick_get_model();
}

static void platform_cleanup()
{
    wb_robot_cleanup();
}

static Dynamics::pose_t platform_get_vehicle_pose()
{
    const double * xyz = wb_gps_get_values(_gps);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(_imu);

    return { xyz[0], -xyz[1], xyz[2], rpy[0], rpy[1], -rpy[2] };
}

static void platform_send_siminfo(const siminfo_t & siminfo)
{
    wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));
}

static int platform_joystick_get_axis_value(const uint8_t axis)
{
    return wb_joystick_get_axis_value(axis);
}

static int platform_joystick_get_pressed_button()
{
    return wb_joystick_get_pressed_button();
}

static joystick_t platform_joystick_get_info() 
{
    return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
}

static int platform_joystick_get_number_of_axes()
{
    return wb_joystick_get_number_of_axes();
}

static int platform_keyboard_get_key()
{
    return wb_keyboard_get_key();
}

static float platform_get_framerate()
{
    return 1000 / _timestep;
}

static int platform_keyboard_down()
{
    return WB_KEYBOARD_DOWN;
}

static int platform_keyboard_left()
{
    return WB_KEYBOARD_LEFT;
}

static int platform_keyboard_right()
{
    return WB_KEYBOARD_RIGHT;
}

static int platform_keyboard_up()
{
    return WB_KEYBOARD_UP;
}

// Code that should work with other platforms --------------------------------

typedef enum {

    JOYSTICK_NONE,
    JOYSTICK_UNRECOGNIZED,
    JOYSTICK_RECOGNIZED

} joystickStatus_e;

typedef enum {

    TOGGLE_HOVER,
    TOGGLE_AUTO

} toggle_e;

static constexpr float ZDIST_LANDING_MAX_M = 0.01;

static Dynamics::pose_t _startingPose;

static mode_e _mode;

static joystickStatus_e getJoystickStatus(void)
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

static void switchMode(toggle_e toggle)
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

static demands_t getSetpointFromKey(const int key)
{
    const float thrust = key == 'W' ? +1 : key == 'S' ? -1 : 0;

    const float roll =
        key == platform_keyboard_right() ? +1.0 :
        key == platform_keyboard_left() ? -1.0 :
        0;

    const float pitch =
        key == platform_keyboard_up() ? +1.0 :
        key == platform_keyboard_down() ? -1.0 :
        0;

    const float yaw = key == 'E' ? +0.5 : key == 'Q' ? -0.5 : 0;

    return demands_t {thrust, roll, pitch, yaw};
}

static void checkKeyboardToggle(
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

static float normalizeJoystickAxis(const int32_t rawval)
{
    return 2.0f * rawval / UINT16_MAX; 
}

static int32_t readJoystickRaw(const int8_t index)
{
    const auto axis = abs(index) - 1;
    const auto sign = index < 0 ? -1 : +1;
    return sign * platform_joystick_get_axis_value(axis);
}

static float readJoystickAxis(const int8_t index)
{
    return normalizeJoystickAxis(readJoystickRaw(index));
}


static void reportJoystick(void)
{
    printf("Unrecognized joystick '%s' with axes ",
            platform_joystick_get_model()); 

    for (uint8_t k=0; k<platform_joystick_get_number_of_axes(); ++k) {

        printf("%2d=%+6d |", k+1, platform_joystick_get_axis_value(k));
    }
}

static void sendSimInfo(siminfo_t & siminfo)
{
    // Grab starting pose first time around
    if (_startingPose.x == INFINITY) {
        _startingPose = platform_get_vehicle_pose();
    }

    memcpy(&siminfo.startingPose, &_startingPose, sizeof(Dynamics::pose_t));
    siminfo.framerate = platform_get_framerate();
    platform_send_siminfo(siminfo);
}

static void getSimInfoFromKeyboard(const bool autonomous, siminfo_t & siminfo)
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

        siminfo.setpoint = getSetpointFromKey(key);
    }

    siminfo.mode = _mode;
}

static bool checkButtonToggle(
        const int button,
        const int target,
        const toggle_e toggle,
        const bool button_was_down)
{
    if (button == target) {
        return true;
    }
    else {
        if (button_was_down) {
            switchMode(toggle);
        }
        return false;
    }
}

static void getSimInfoFromJoystick(siminfo_t & siminfo, const bool autonomous)
{
    static bool _hover_button_was_down;
    static bool _auto_button_was_down;

    auto axes = platform_joystick_get_info();

    const auto button = platform_joystick_get_pressed_button();

    _hover_button_was_down =
        checkButtonToggle(button, 5, TOGGLE_HOVER, _hover_button_was_down);

    _auto_button_was_down =
        checkButtonToggle(button, 4, TOGGLE_AUTO, _auto_button_was_down);

    siminfo.mode = _mode;

    if (!autonomous) {

        siminfo.setpoint.pitch = readJoystickAxis(axes.pitch);
        siminfo.setpoint.roll = readJoystickAxis(axes.roll);
        siminfo.setpoint.yaw = readJoystickAxis(axes.yaw);

        siminfo.setpoint.thrust = readJoystickAxis(axes.throttle);
    }
}


int main(int argc, char ** argv) 
{
    (void)argc;

    const char * worldname =  argv[1];
    const char * logfilename =  argv[2];

    // OOB value to trigger initialization in step()
    _startingPose.x = INFINITY;

    _mode = MODE_IDLE;

    demands_t * autonomousSetpoint = nullptr;

    platform_init();

    while (true) {

        siminfo_t siminfo = {};
        strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
        strcpy(siminfo.worldname, worldname);
        strcpy(siminfo.logfilename, logfilename);

        if (!platform_step()) {
            break;
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
                getSimInfoFromKeyboard(autonomous, siminfo);
        }

        if (autonomous) {
            memcpy(&siminfo.setpoint, autonomousSetpoint, sizeof(demands_t));
        }

        // On descent, switch mode to idle when close enough to ground
        const auto pose = platform_get_vehicle_pose();
        if (_mode == MODE_LANDING &&
                (pose.z -_startingPose.z ) < ZDIST_LANDING_MAX_M) {
            _mode = MODE_IDLE;
        }

        sendSimInfo(siminfo);
    }

    platform_cleanup();

    return 0;
}
