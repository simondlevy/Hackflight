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
#include <unistd.h>
using namespace std;

// Hackflight
#include <setpoint/multiranger.hpp>

// Webots
#include <webots/range_finder.h>

// Misc.
#include "../support.hpp"

static WbDeviceTag _ranger;

static void readRanger(const int width, const int height,
        int16_t * distance_mm) 
{
    const float * image = wb_range_finder_get_range_image(_ranger);

    for (int x=0; x<width; ++x) {

        for (int y=0; y<height; ++y) {

            const float distance_m =
                wb_range_finder_image_get_depth(image, width, x, y);

            distance_mm[y*width+x] = isinf(distance_m) ? -1 :
                (int16_t)(1000 * distance_m);
        }
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

    if (flightMode == MODE_HOVERING) {

        getSetpointFromKey(key, siminfo);
    }

    siminfo.flightMode = flightMode;
}

static bool flight_mode_hovering(const flightMode_t mode)
{
    return mode == MODE_HOVERING;
}

static bool step(const string worldname, const setpointType_e setpointType,
        FILE * logfp)
{
    if (wb_robot_step(_timestep) == -1) {
        return false;
    }

    static flightMode_t _flightMode;

    Simulator::info_t siminfo = {};

    strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
    strcpy(siminfo.worldname, worldname.c_str());

    int16_t ranger_distance_mm[1000] = {}; // arbitrary max size

    const int width = wb_range_finder_get_width(_ranger);
    const int height = wb_range_finder_get_height(_ranger);

    readRanger(width, height, ranger_distance_mm);

    //rv.show(ranger_distance_mm, LIDAR_DISPLAY_SCALEUP);

    switch (getJoystickStatus()) {

        case JOYSTICK_RECOGNIZED:
            getSimInfoFromJoystick(siminfo, _flightMode, flight_mode_hovering);
            break;

        case JOYSTICK_UNRECOGNIZED:
            reportJoystick();
            // fall thru

        default:
            getSimInfoFromKeyboard(siminfo, _flightMode);
    }

    if (setpointType == SETPOINT_LIDAR) {
        MultiRanger::getSetpoint(8, 8, ranger_distance_mm, siminfo.setpoint);
    }

    // On descent, switch mode to idle when close enough to ground
    const auto z = wb_gps_get_values(_gps)[2] - _start_z; 
    if (_flightMode == MODE_LANDING && z <= Dynamics::ZMIN) {
        _flightMode = MODE_IDLE;
    }

    if (z > 0.18) {
        fprintf(logfp, "%d\n", ranger_distance_mm[0]);
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

    const std::string worldname =  argv[1];
    const std::string setpoint =  argv[2];

    setpointType_e setpointType = SETPOINT_HUMAN;

    if (setpoint == "lidar") {
    }
    else if (setpoint == "human") {
    }
    else {
        printf("Unrecognized setpoint '%s'; defaulting to human\n", setpoint.c_str());
    }

    wb_robot_init();

    _timestep = wb_robot_get_basic_time_step();

    _emitter = wb_robot_get_device("emitter");

    _ranger = wb_robot_get_device("range-finder");
    wb_range_finder_enable(_ranger, _timestep);

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

    FILE * logfp = fopen("/home/levys/Desktop/hackflight/webots/controllers/"
            "experimental/groundtruth.csv", "w");

    while (true) {

        if (!step(worldname, setpointType, logfp)) {
            break;
        }
    }

    wb_robot_cleanup();

    return 0;
}
