/*
   C++ flight simulator spiking-neural net controller for Hackflight

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

#include <webots.hpp>
#include <snn.hpp>
#include <utils.hpp>

static const bool USE_NETWORK = true;

// Small scaling value relating climb-rate demand to throttle stick
static const float THROTTLE_SCALE = 0.2;

// We consider altitudes below this value to be the ground
static const float ZGROUND = 0.05;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

static const float TAKEOFF_TIME = 2;

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

// Non-neuro Utils::pcontrol(ler ---------------

static const float K_CLIMBRATE = 25;

// ------------------------------------

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    SNN * snn = NULL;

    try {
        snn = new SNN("networks/hover_risp.txt", "risp");
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

    uint32_t tick = 0;

    bool reached_altitude = false;

    while (true) {

        hf::state_t state = {};

        hf::demands_t stickDemands = {};

        if (!sim.step(stickDemands, state)) {
            break;
        }

        const auto scaledThrottle = THROTTLE_SCALE * stickDemands.thrust;

        // Get current climb rate observation
        const auto dz = state.dz;

        const double time = tick * timestep / 1000;

        if (time > TAKEOFF_TIME) {
            reached_altitude = true;
        }

        (void)snn;
        /*
        vector<double> o = {_ztarget, z, dz};
        vector <double> a;
        snn->getActions(o, a);
        const auto motor_snn = a[0];
        (void)motor_snn;*/

        const auto dz_target = scaledThrottle;

        const auto thrust = K_CLIMBRATE *  (dz_target - dz);

        const auto motor_old = THRUST_BASE + thrust;

        const auto motor = reached_altitude  ? motor_old : THRUST_TAKEOFF;

        tick++;

        // Run the motors
        wb_motor_set_velocity(_motor1, +motor);
        wb_motor_set_velocity(_motor2, -motor);
        wb_motor_set_velocity(_motor3, +motor);
        wb_motor_set_velocity(_motor4, -motor);
    }

    wb_robot_cleanup();

    return 0;
}
