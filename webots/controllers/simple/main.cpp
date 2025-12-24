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


// Misc.
#include "../support.hpp"

static const uint8_t LIDAR_DISPLAY_SCALEUP = 64;

static void getSimInfoFromJoystick(Simulator::info_t & siminfo, flightMode_t & flightMode)
{
    static bool _hover_button_was_down;
    static bool _auto_button_was_down;

    auto axes = getJoystickInfo();

    const auto button = wb_joystick_get_pressed_button();

    checkButtonToggle(button, 5, TOGGLE_HOVER, _hover_button_was_down, flightMode);

    checkButtonToggle(button, 4, TOGGLE_AUTO, _auto_button_was_down, flightMode);

    siminfo.flightMode = flightMode;

    if (siminfo.flightMode != MODE_IDLE) {

        siminfo.setpoint.pitch = readJoystickAxis(axes.pitch);
        siminfo.setpoint.roll = readJoystickAxis(axes.roll);
        siminfo.setpoint.yaw = readJoystickAxis(axes.yaw);

        climb(readJoystickAxis(axes.throttle));
    }
}

static void getSimInfoFromKeyboard(
        Simulator::info_t & siminfo, flightMode_t & flightMode)
{
    static bool _enter_was_down;
    static bool _spacebar_was_down;

    const int key = wb_keyboard_get_key();

    if (key == -1 ) {
        _enter_was_down = false;
        _spacebar_was_down = false;
    }

    checkKeyboardToggle(key, 4, TOGGLE_HOVER, _enter_was_down, flightMode);

    checkKeyboardToggle(key, 32, TOGGLE_AUTO, _spacebar_was_down, flightMode);

    if (flightMode != MODE_IDLE) {

        getSetpointFromKey(key, siminfo);
    }

    siminfo.flightMode = flightMode;
}
static bool step()
{
    if (wb_robot_step(_timestep) == -1) {
        return false;
    }

    static flightMode_t _flightMode;

    Simulator::info_t siminfo = {};

    switch (getJoystickStatus()) {

        case JOYSTICK_RECOGNIZED:
            getSimInfoFromJoystick(siminfo, _flightMode);
            break;

        case JOYSTICK_UNRECOGNIZED:
            reportJoystick();
            // fall thru

        default:
            getSimInfoFromKeyboard(siminfo, _flightMode);
    }

    // On descent, switch mode to idle when close enough to ground
    const auto z = wb_gps_get_values(_gps)[2] - _start_z; 
    if (_flightMode == MODE_LANDING && z <= Dynamics::ZMIN) {
        _flightMode = MODE_IDLE;
    }

    sendSimInfo(siminfo);

    return true;
}

static void animateMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);
    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction * 60);
}

int main(int argc, char ** argv) 
{
    (void)argc;

    const std::string world =  argv[1];
    const std::string setpoint =  argv[2];

    wb_robot_init();

    _timestep = wb_robot_get_basic_time_step();

    _emitter = wb_robot_get_device("emitter");

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

    while (true) {

        if (!step()) {
            break;
        }
    }

    wb_robot_cleanup();

    return 0;
}
