/*
  C++ flight simulator takeoff example for Hackflight
 
  Copyright (C) 2024 Simon D. Levy
 
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

#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>

// Control constants
static const float K1 = 56;
static const float K2 = 25;
static const float K3 = 2;

static const float ZTARGET = 0.2;

static const float DT = 0.01;

static const float PLOT_TIME_LIMIT = 7;

static const float SPINUP_TIME = 1.5;

static const float SPIN_SCALEUP = 2;

// Motors
static WbDeviceTag _motor1;
static WbDeviceTag _motor2;
static WbDeviceTag _motor3;
static WbDeviceTag _motor4;

static WbDeviceTag _makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

int main(int argc, char ** argv)
{
    wb_robot_init();

    _motor1 = _makeMotor("motor1", +1);
    _motor2 = _makeMotor("motor2", -1);
    _motor3 = _makeMotor("motor3", +1);
    _motor4 = _makeMotor("motor4", -1);

    const int timestep = (int)wb_robot_get_basic_time_step();

    auto gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);

    // Start with faked-up initial motor to animate motor spin-up
    auto motor = K2 + K3;

    float zprev = 0;

    while (wb_robot_step(timestep) != -1) {

        // Get current altitude
        const auto z = wb_gps_get_values(gps)[2];
        
        const auto motor =
            K1 + K2 * (K3 * (ZTARGET - z) - ((z - zprev) / DT));

        zprev = z;

        // Animate the motors
        wb_motor_set_velocity(_motor1, +motor);
        wb_motor_set_velocity(_motor2, -motor);
        wb_motor_set_velocity(_motor3, +motor);
        wb_motor_set_velocity(_motor4, -motor);
    }

    wb_robot_cleanup();

    return 0;
}
