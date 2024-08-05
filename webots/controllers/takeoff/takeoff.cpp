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

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>

#include <snn.hpp>

static const bool USE_NETWORK = true;

// Time constant for computing climb rate
static const float DT = 0.01;

// For training
static const float ZTARGET = 0.2;
static const float TBASE = 56;

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

// Non-neuro controller ---------------

static const float K_ALTITUDE = 2;
static const float K_CLIMBRATE = 25;

// ------------------------------------

static float control(const float k, const float target, const float actual)
{
    return k * (target - actual);
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    SNN * snn = NULL;

    try {
        snn = new SNN("networks/takeoff_risp_best.txt", "risp");
    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    wb_robot_init();

    _motor1 = _makeMotor("motor1", +1);
    _motor2 = _makeMotor("motor2", -1);
    _motor3 = _makeMotor("motor3", +1);
    _motor4 = _makeMotor("motor4", -1);

    const int timestep = (int)wb_robot_get_basic_time_step();

    auto gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);

    float zprev = 0;

    FILE * logfp = fopen("altitude.csv", "w");
    fprintf(logfp, "z, dz_target\n");

    while (wb_robot_step(timestep) != -1) {

        // Get current altitude and climb rate observations
        const auto z = wb_gps_get_values(gps)[2];
        const auto dz = (z - zprev) / DT;
        zprev = z;

        float motor = 0;

        if (USE_NETWORK) {
            vector<double> o = {z, dz};
            vector <double> a;
            snn->getActions(o, a);
            motor = a[0];
        }

        else {
               const auto dz_target = control(K_ALTITUDE, ZTARGET, z); 
               fprintf(logfp, "%f,%f\n", z, dz_target);
               fflush(logfp);
               const auto thrust = control(K_CLIMBRATE,  dz_target, dz);
               motor = thrust + TBASE;
        }

        // Run the motors
        wb_motor_set_velocity(_motor1, +motor);
        wb_motor_set_velocity(_motor2, -motor);
        wb_motor_set_velocity(_motor3, +motor);
        wb_motor_set_velocity(_motor4, -motor);
    }

    wb_robot_cleanup();

    fclose(logfp);

    return 0;
}
