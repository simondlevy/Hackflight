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

#include <webots.hpp>
#include <snn.hpp>

static const bool USE_NETWORK = false;

// Time constant for computing climb rate
static const float DT = 0.01;

static const float INITIAL_ALTITUDE_TARGET = 0.2;

// We consider throttle inputs above this below this value to be
// positive for takeoff
static constexpr float THROTTLE_ZERO = 0.05;

static constexpr float THROTTLE_SCALE = 0.005;

// We consider altitudes below this value to be the ground
static constexpr float ZGROUND = 0.05;


static const float TBASE = 56;

typedef enum {

    STATUS_LANDED,
    STATUS_TAKING_OFF,
    STATUS_FLYING

} flyingStatus_e;


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

    hf::Simulator sim = {};

    sim.init();

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

    uint32_t tick = 0;

    while (true) {

        hf::state_t state = {};

        hf::demands_t stickDemands = {};

        if (!sim.step(stickDemands, state)) {
            break;
        }

        static flyingStatus_e _status;

        static float _altitude_target;

        _altitude_target =
            _status == STATUS_FLYING ? 
            _altitude_target + THROTTLE_SCALE * stickDemands.thrust :
            _status == STATUS_LANDED ?
            INITIAL_ALTITUDE_TARGET :
            _altitude_target;

        _status = 

            _status == STATUS_TAKING_OFF  && state.z > ZGROUND ?  
            STATUS_FLYING :

            _status == STATUS_FLYING && state.z <= ZGROUND ?  
            STATUS_LANDED :

            _status == STATUS_LANDED && 
            stickDemands.thrust > THROTTLE_ZERO ? 
            STATUS_TAKING_OFF :

            _status;

        const auto landed = _status == STATUS_LANDED;

        // Get current altitude and climb rate observations
        const auto z = state.z;
        const auto dz = state.dz;

        float motor = 0;

        if (USE_NETWORK) {
            vector<double> o = {z, dz};
            vector <double> a;
            snn->getActions(o, a);
            motor = a[0];
        }

        else {
               const auto dz_target = control(K_ALTITUDE, _altitude_target, z);
               const auto thrust = control(K_CLIMBRATE,  dz_target, dz);
               motor = landed ? 0 : TBASE + thrust;
        }

        printf("%f %f %f %f %f\n", 
                tick * timestep / 1000., _altitude_target, z, dz, motor);

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
