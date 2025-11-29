/* 
 * Custom physics plugin support for Hackflight Webots-based simulator
 *
 *  Copyright (C) 2025 Simon D. Levy
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

#include <stdio.h>

#include <plugins/physics.h>

#include <sim_control.hpp>
#include <num.hpp>
#include <mixers/crazyflie.hpp>
#include <dynamics.hpp>
#include <vehicles/diyquad.hpp>

static const float DYNAMICS_RATE = 100000; // Hz

static const int PID_UPDATE_RATE = 1024; // Plank

static Dynamics _dynamics = Dynamics(VPARAMS, 1./DYNAMICS_RATE);

static void reportStatus(const siminfo_t & siminfo)
{
    const char * str[6] = {
        "IDLE",
        "ARMED",
        "HOVERING",
        "AUTONOMOUS",
        "LANDING",
        "LOST_CONTACT"
    };

    dWebotsConsolePrintf("status=%s",
            str[siminfo.status]);
}

static pose_t run_sim_middle_loop(const siminfo_t & siminfo)
{
    bool landed = false;

    // Run control in middle loop
    for (uint32_t j=0;
            j < (uint32_t)(1 / siminfo.framerate * PID_UPDATE_RATE);  ++j) {

        const auto d = _dynamics;

        const vehicleState_t state =  {
            0, // x
            d.state.dx,
            0, // y
            d.state.dy,
            d.state.z,                     
            d.state.dz,                   
            Num::RAD2DEG* d.state.phi, 
            Num::RAD2DEG* d.state.dphi, 
            Num::RAD2DEG* d.state.theta, 
            Num::RAD2DEG* d.state.dtheta,
            Num::RAD2DEG* d.state.psi,   
            Num::RAD2DEG* d.state.dpsi
        };

        demands_t demands = {};

        (void)reportStatus/*(siminfo)*/;

        _closedLoopControl.run(
                1 / (float)PID_UPDATE_RATE,
                siminfo.status == STATUS_HOVERING,
                state,
                siminfo.demands,
                demands);

        demands.roll *= Num::DEG2RAD;
        demands.pitch *= Num::DEG2RAD;
        demands.yaw *= Num::DEG2RAD;

        float motors[4] = {};

        Mixer::mix(demands, motors);

        // Run dynamics in innermost loop
        for (uint32_t k=0; k<DYNAMICS_RATE / PID_UPDATE_RATE; ++k) {

            _dynamics.update(motors,
                    Mixer::rotorCount,
                    Mixer::roll,
                    Mixer::pitch,
                    Mixer::yaw);

            if (_dynamics.state.z < 0) {
                landed = true;
                break;
            }
        }
    }

    // Reset dynamics if landed
    if (landed) {
        _dynamics.reset();
    }

    // Get current pose from dynamics
    return _dynamics.getPose();
}

static constexpr char ROBOT_NAME[] = "diyquad";

static dBodyID _robotBody;

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

    _closedLoopControl.init();
}

// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (_robotBody == NULL) {
        return;
    }

    int size = 0;

    siminfo_t siminfo = {};

    const auto buffer = (siminfo_t *)dWebotsReceive(&size);

    if (size == sizeof(siminfo_t)) {
        memcpy(&siminfo, buffer, sizeof(siminfo));
    }

    // This happens at startup
    if (siminfo.framerate == 0) {
        return;
    }

    // Run controllers in middle loop, dynamics inside that
    const pose_t pose = run_sim_middle_loop(siminfo);

    // Turn Euler angles into quaternion, negating psi for nose-right positive 
    const axis3_t euler = { pose.phi, pose.theta, -pose.psi};
    axis4_t quat = {};
    Num::euler2quat(euler, quat);
    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
    dBodySetQuaternion(_robotBody, q);

    // Set robot posed based on state and starting position, negating for
    // rightward negative
    dBodySetPosition(
            _robotBody,
            siminfo.start_x + pose.x,
            siminfo.start_y - pose.y,
            siminfo.start_z + pose.z);
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
