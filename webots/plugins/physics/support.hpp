/* 
 * Custom physics plugin support for Hackflight simulator
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

// Webots
#include <plugins/physics.h>

// Hackflight
#include <hackflight.hpp>
#include <sim/dynamics.hpp>
#include <sim/sensors/optical_flow.hpp>
#include <sim/sensors/gyrometer.hpp>
#include <sim/sensors/rangefinder.hpp>
#include <mixers/bfquadx.hpp>

static const uint32_t DYNAMICS_FREQ = 1e5; // Hz

static const uint32_t PID_FREQ = 1e3; // Hz

static const char ROBOT_NAME[] = "diyquad";

static dBodyID _robotBody;

static hf::Dynamics::vehicle_params_t diyquad_params = {

    1.0e-1, // mass [kg]
    5.0e-2, // arm length L [m]

    3.6e-5, // force coefficient B [F=b*w^2]
    7.0e-6, // drag coefficient D [T=d*w^2]
    2.0e-5  // I [kg*m^2]   // pitch, roll
};


static auto dynamics = hf::Dynamics(diyquad_params, 1./DYNAMICS_FREQ);

static void setPose(hf::Dynamics & dynamics)
{
    // Get current pose from dynamics
    const auto pose = dynamics.getPose();

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const hf::axis3_t euler = { pose.phi, pose.theta, -pose.psi };
    hf::axis4_t quat = {};
    hf::Utils::euler2quat(euler, quat);
    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robotBody, q);

    // Set robot posed based on state, negating for rightward negative
    dBodySetPosition(_robotBody, pose.x, -pose.y, pose.z);
}

static void updateDynamics(const hf::demands_t & demands)
{
    hf::BfQuadXMixer mixer = {};

    float motors[4] = {};

    mixer.run(demands, motors);

    for (uint32_t k=0; k<DYNAMICS_FREQ / PID_FREQ; ++k) {

        dynamics.update(motors, &mixer);
    }
}

static bool getSimInfo(hf::siminfo_t & siminfo)
{
    if (_robotBody == NULL) {
        return false;
    }

    int size = 0;

    const auto buffer = (hf::siminfo_t *)dWebotsReceive(&size);

    if (size == sizeof(hf::siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }


    // This happens at startup
    if (siminfo.framerate == 0) {
        return false;
    }

    return true;
}

static uint32_t outerLoopCount(const hf::siminfo_t & siminfo)
{
    return (uint32_t)(1 / siminfo.framerate * PID_FREQ);
}

static float pidDt()
{
    return 1. / PID_FREQ;
}

static hf::state_t estimateState()
{
    // Get simulated gyro values
    const auto gyro = hf::Gyro::read(dynamics);

    // Get simulated rangefinder distance
    const auto h = hf::Rangefinder::read(dynamics);

    // Get simulated optical flow
    const auto flow = hf::OpticalFlow::read(dynamics, h);

    // Cheat on Euler angles for now (should get them by fusing gyro and accel)
    const auto pose = dynamics.getPose();

    // Turn rangefinder distance directly into altitude 
    const auto z = h * (cos(pose.phi) * cos(pose.theta)) / 1000; // mm => m

    // XXX Cheat on remaining sensors for now
    const auto dxdy = dynamics.getGroundTruthHorizontalVelocities();
    const auto dz = dynamics.getGroundTruthVerticalVelocity();
    const auto r = hf::Utils::RAD2DEG;

     // https://www.bitcraze.io/documentation/repository/crazyflie-firmware/                                      //  master/images/flowdeck_velocity.png

    const float dy = flow.y;

    dWebotsConsolePrintf("dy=%+3.3e (%+3.3e)\n", dxdy.y, dy);

    return hf::state_t {pose.x, dxdy.x, pose.y, dxdy.y, z, dz,
            r * pose.phi, gyro.x, r * pose.theta, gyro.y, r * pose.psi, gyro.z
    };
}


DLLEXPORT void webots_physics_init() 
{
    // init global variables
    _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

    if (_robotBody == NULL) {

        dWebotsConsolePrintf("quadrotor_physics :: webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robotBody, 0);
    }
}


DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) 
{
    (void)g1;
    (void)g2;

    return 0;
}

DLLEXPORT void webots_physics_cleanup() 
{
}
