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

#include <stdio.h>

// Webots
#include <plugins/physics.h>

// Hackflight
#include <hackflight.hpp>
#include <mixers/bfquadx.hpp>
#include <sim/dynamics.hpp>
#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>


class SimHelper {

    public:

        static const uint32_t DYNAMICS_FREQ = 1e5; // Hz

        static constexpr float MOTOR_HOVER = 74.565; // rad/sec

        static const uint32_t PID_FREQ = 1e3; // Hz

        static constexpr char ROBOT_NAME[] = "diyquad";

        dBodyID _robotBody;

        hf::Dynamics::vehicle_params_t _diyquad_params = {

            1.0e-1, // mass [kg]
            5.0e-2, // arm length L [m]

            3.6e-5, // force coefficient B [F=b*w^2]
            7.0e-6, // drag coefficient D [T=d*w^2]
            2.0e-5  // I [kg*m^2]   // pitch, roll
        };

        hf::AltitudePid _altitudePid;

        hf::PitchRollAnglePid _pitchRollAnglePid;

        hf::PitchRollRatePid _pitchRollRatePid;

        hf::YawRatePid _yawRatePid;


        hf::Dynamics _dynamics =
            hf::Dynamics(_diyquad_params, 1./DYNAMICS_FREQ);

        void setPose()
        {
            // Get current pose from dynamics
            const auto pose = _dynamics.getPose();

            // Turn Euler angles into quaternion, negating psi for nose-right
            // positive 
            const hf::axis3_t euler = { pose.phi, pose.theta, -pose.psi };
            hf::axis4_t quat = {};
            hf::Utils::euler2quat(euler, quat);
            const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};
            dBodySetQuaternion(_robotBody, q);

            // Set robot posed based on state, negating for rightward negative
            dBodySetPosition(_robotBody, pose.x, -pose.y, pose.z);
        }

        void updateDynamics(const hf::demands_t & demands)
        {
            hf::BfQuadXMixer mixer = {};

            float motors[4] = {};

            mixer.run(demands, motors);

            for (uint32_t k=0; k<DYNAMICS_FREQ / PID_FREQ; ++k) {

                _dynamics.update(motors, &mixer);
            }
        }

        bool getSimInfo(hf::siminfo_t & siminfo)
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

        hf::state_t getGroundTruthState()
        {
            return hf::state_t {
                _dynamics._x1,
                    _dynamics._x2 * cos(_dynamics._x11) -
                        _dynamics._x4 * sin(_dynamics._x11),
                    _dynamics._x3,
                    -(_dynamics._x2 * sin(_dynamics._x11) +
                            _dynamics._x4 * cos(_dynamics._x11)),
                    _dynamics._x5,
                    _dynamics._x6,
                    hf::Utils::RAD2DEG* _dynamics._x7,
                    hf::Utils::RAD2DEG* _dynamics._x8,
                    hf::Utils::RAD2DEG* _dynamics._x9,
                    hf::Utils::RAD2DEG* _dynamics._x10,
                    hf::Utils::RAD2DEG* _dynamics._x11,
                    hf::Utils::RAD2DEG* _dynamics._x12,
            };
        }

        static uint32_t outerLoopCount(const hf::siminfo_t & siminfo)
        {
            return (uint32_t)(1 / siminfo.framerate * PID_FREQ);
        }

        static float pidDt()
        {
            return 1. / PID_FREQ;
        }

};

static const uint32_t DYNAMICS_FREQ = 1e5; // Hz

static const float MOTOR_HOVER = 74.565; // rad/sec

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
    void setup_controllers();
    setup_controllers();
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
