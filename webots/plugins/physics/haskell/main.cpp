/* 
 * Custom physics plugin for Hackflight simulator
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

// Webots
#include <plugins/physics.h>

// Hackflight
#include <hackflight.hpp>
#include <sim/dynamics.hpp>
#include <mixers/bfquadx.hpp>

static const uint32_t DYNAMICS_FREQ = 1e5; // Hz

static const uint32_t PID_FREQ = 1e3; // Hz

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_POST_SCALE = 50;

static const char ROBOT_NAME[] = "quadrotor";

static const float MOTOR_HOVER = 74.565; // 55.385; // rad/sec

static dBodyID _robotBody;

static hf::Dynamics::vehicle_params_t tinyquad_params = {

    1.0e-1, // mass [kg]
    5.0e-2, // arm length L [m]

    3.6e-5, // force coefficient B [F=b*w^2]
    7.0e-6, // drag coefficient D [T=d*w^2]
    2.0e-5  // I [kg*m^2]   // pitch, roll
};


static hf::siminfo_t getSimInfo()
{
    static hf::siminfo_t _siminfo;

    int size = 0;

    const auto buffer = (hf::siminfo_t *)dWebotsReceive(&size);

    if (size == sizeof(hf::siminfo_t)) {
        memcpy(&_siminfo, buffer, sizeof(_siminfo));
    }

    return _siminfo;
}

static auto dynamics = hf::Dynamics(tinyquad_params, 1./DYNAMICS_FREQ);

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

DLLEXPORT void webots_physics_step() 
{
    if (_robotBody == NULL) {
        return;
    }

    const auto siminfo = getSimInfo();

    // This happens at startup
    if (siminfo.framerate == 0) {
        return;
    }

    // Run control in outer loop
    for (uint32_t j=0; j < (1 / siminfo.framerate * PID_FREQ); ++j) {
        
        // Start with open-loop demands
        hf::demands_t demands = {
            siminfo.demands.thrust,
            siminfo.demands.roll,
            siminfo.demands.pitch,
            siminfo.demands.yaw
        };

        // Throttle-down should reset pids
        const auto resetPids = demands.thrust < THROTTLE_DOWN;

        // Get simulated gyro
        const auto gyro = dynamics.readGyro();

        // XXX Cheat on remaining sensors for now
        const auto pose = dynamics.getPose();
        const auto dxdy = dynamics.getGroundTruthHorizontalVelocities();
        const auto dz = dynamics.getGroundTruthVerticalVelocity();

        const auto r = hf::Utils::RAD2DEG;
        const auto state = hf::state_t {
                pose.x,
                dxdy.x,
                pose.y,
                dxdy.y,
                pose.z,
                dz,
                r * pose.phi,
                gyro.x,
                r * pose.theta,
                gyro.y,
                r * pose.psi,
                gyro.z
        };

        // Run PID controllers to get final demands
        
        static const float pid_dt  = 1. / PID_FREQ;

        // Run mixer to get motors spins from demands
        hf::BfQuadXMixer mixer = {};
        float motors[4] = {};
        mixer.run(demands, motors);

        // Run dynamics in inner loop to update state with motors
        for (uint32_t k=0; k<DYNAMICS_FREQ / PID_FREQ; ++k) {
            dynamics.update(motors, &mixer);
        }
    }

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

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) 
{
    (void)g1;
    (void)g2;

    return 0;
}

DLLEXPORT void webots_physics_cleanup() 
{
}
