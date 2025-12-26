/* 
   C++ flight simulator support for Hackflight using Webots

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

#include <simulator/support.hpp>

#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

static WbDeviceTag _emitter;
static WbDeviceTag _gps;
static WbDeviceTag _ranger;

static double _timestep;

static void startMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);
    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction * 60);
}

float Support::platform_get_time()
{
    return wb_robot_get_time();
}

void Support::platform_get_vehicle_location(double & x, double & y, double & z)
{
    const double * xyz = wb_gps_get_values(_gps);

    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
}

void Support::platform_send_siminfo(const Simulator::info_t & siminfo)
{
    wb_emitter_send(_emitter, &siminfo, sizeof(siminfo));
}

int Support::platform_joystick_get_axis_value(const uint8_t axis)
{
    return wb_joystick_get_axis_value(axis);
}

int Support::platform_joystick_get_pressed_button()
{
    return wb_joystick_get_pressed_button();
}

Support::joystick_t Support::platform_joystick_get_info() 
{
    return JOYSTICK_AXIS_MAP[wb_joystick_get_model()];
}

const char * Support::platform_joystick_get_model()
{
    return wb_joystick_get_model();
}

int Support::platform_joystick_get_number_of_axes()
{
    return wb_joystick_get_number_of_axes();
}

int Support::platform_keyboard_get_key()
{
    return wb_keyboard_get_key();
}

void Support::platform_init()
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

void Support::platform_cleanup()
{
    wb_robot_cleanup();
}

bool Support::platform_step()
{
    return wb_robot_step(_timestep) != -1;
}

float Support::platform_get_framerate()
{
    return 1000 / _timestep;
}

void Support::platform_read_rangefinder(int16_t * distance_mm) 
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
        
int Support::platform_keyboard_down()
{
    return WB_KEYBOARD_DOWN;
}
        
int Support::platform_keyboard_left()
{
    return WB_KEYBOARD_LEFT;
}

int Support::platform_keyboard_right()
{
    return WB_KEYBOARD_RIGHT;
}

int Support::platform_keyboard_up()
{
    return WB_KEYBOARD_UP;
}
