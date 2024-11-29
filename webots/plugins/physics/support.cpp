/* 
 * Custom physics plugin for Hackflight simulator using C++ PID controllers
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
#include <mixers/bfquadx.hpp>
#include <sim/dynamics.hpp>
#include <sim/vehicles/diyquad.hpp>

static const float DYNAMICS_RATE = 100000; // Hz

static const float PID_RATE = 1000; // Hz

static constexpr char ROBOT_NAME[] = "diyquad";

// Write these for each paradigm (standard, snn, haskell, ekf) ---------------

void setup_controllers();

hf::state_t estimate_state(
        const hf::Dynamics & dynamics, const float pid_rate);

hf::demands_t run_controllers(
        const float pid_dt,
        const hf::siminfo_t & siminfo,
        const hf::state_t & state);

// ---------------------------------------------------------------------------

static dBodyID _robotBody;

static hf::Dynamics _dynamics = hf::Dynamics(hf::VPARAMS, 1./DYNAMICS_RATE);

DLLEXPORT void webots_physics_init() 
{
    _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

    if (_robotBody == NULL) {

        dWebotsConsolePrintf("webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robotBody, 0);
    }

    // Implementation goes in your main
    setup_controllers();
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (_robotBody == NULL) {
        return;
    }

    int size = 0;

    hf::siminfo_t siminfo = {};

    const auto buffer = (hf::siminfo_t *)dWebotsReceive(&size);

    if (size == sizeof(hf::siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return;
    }

    // Run control in middle loop
    for (uint32_t j=0; j < (uint32_t)(1 / siminfo.framerate * PID_RATE);  ++j) {

        const auto state = estimate_state(_dynamics, PID_RATE);

        const auto demands = run_controllers(1 / PID_RATE, siminfo, state);

        hf::BfQuadXMixer mixer = {};

        float motors[4] = {};

        mixer.run(demands, motors);

        // Update dynamics in innermost loop
        for (uint32_t k=0; k<DYNAMICS_RATE / PID_RATE; ++k) {

            _dynamics.update(motors, &mixer);
        }
    }

    // Get current pose from dynamics
    const auto pose = _dynamics.getPose();

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
