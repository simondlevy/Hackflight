/* 
   C++ flight simulator support for Webots

   Copyright (C) 2026 Simon D. Levy

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

#include "platform.h"

#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

// Webots-specific code -------------------------------------------------------

static WbDeviceTag _emitter;
static WbDeviceTag _gps;
static WbDeviceTag _imu;
static WbDeviceTag _ranger;

static double _timestep;

static void startMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);
    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction * 60);
}

static void startMotors()
{
    startMotor("motor1", -1);
    startMotor("motor2", +1);
    startMotor("motor3", +1);
    startMotor("motor4", -1);
}

void platform_init()
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

    wb_robot_step(_timestep);
}

bool platform_step()
{
    return wb_robot_step(_timestep) != -1;
}

const char * platform_joystick_get_model()
{
    return wb_joystick_get_model();
}

void platform_cleanup()
{
    wb_robot_cleanup();
}

double platform_get_vehicle_x()
{
    return wb_gps_get_values(_gps)[0];
}

double platform_get_vehicle_y()
{
    // Negate to make rightward positive
    return -wb_gps_get_values(_gps)[1];
}

double platform_get_vehicle_z()
{
    return wb_gps_get_values(_gps)[2];
}

double platform_get_vehicle_phi()
{
    return wb_inertial_unit_get_roll_pitch_yaw(_imu)[0];
}

double platform_get_vehicle_theta()
{
    return wb_inertial_unit_get_roll_pitch_yaw(_imu)[1];
}

double platform_get_vehicle_psi()
{
    // Negate for nose-right positive
    return -wb_inertial_unit_get_roll_pitch_yaw(_imu)[2];
}

void platform_send_siminfo(const void * info, const size_t size)
{
    wb_emitter_send(_emitter, info, size);
}

int platform_joystick_get_axis_value(const uint8_t axis)
{
    return wb_joystick_get_axis_value(axis);
}

int platform_joystick_get_pressed_button()
{
    return wb_joystick_get_pressed_button();
}

const char * platform_joystick_get_name()
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

float platform_get_framerate()
{
    return 1000 / _timestep;
}

int platform_keyboard_down()
{
    return WB_KEYBOARD_DOWN;
}

int platform_keyboard_left()
{
    return WB_KEYBOARD_LEFT;
}

int platform_keyboard_right()
{
    return WB_KEYBOARD_RIGHT;
}

int platform_keyboard_up()
{
    return WB_KEYBOARD_UP;
}
