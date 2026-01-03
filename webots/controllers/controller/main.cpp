/* 
   C++ flight simulator outerLoop for Hackflight with custom physics plugin

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

#include <string.h>

#include <unistd.h>
using namespace std;

#include <simulator/outer.hpp>

#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/joystick.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>

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

int main(int argc, char ** argv) 
{
    (void)argc;

    const std::string worldname =  argv[1];

    SimOuterLoop outerLoop = {};

    outerLoop.begin();

    while (true) {

        siminfo_t siminfo = {};
        strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
        strcpy(siminfo.worldname, worldname.c_str());

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

void SimOuterLoop::platform_get_vehicle_pose(pose_t & pose)
{
    const double * xyz = wb_gps_get_values(_gps);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(_imu);

    pose.x = xyz[0];
    pose.y = -xyz[1]; // negate Y for leftward positive
    pose.z = xyz[2];
    pose.phi = rpy[0];
    pose.theta = rpy[1];
    pose.psi = rpy[2];
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

    startMotor("motor1", -1);
    startMotor("motor2", +1);
    startMotor("motor3", +1);
    startMotor("motor4", -1);

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
