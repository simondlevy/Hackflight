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

static bool flight_mode_not_idle(const flightMode_t mode)
{
    return mode != MODE_IDLE;
}

static bool step()
{
    static flightMode_t _flightMode;

    Simulator::info_t siminfo = {};

    if (!beginStep(flight_mode_not_idle, _flightMode, siminfo)) {
        return false;
    }

    endStep(siminfo, _flightMode);

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
