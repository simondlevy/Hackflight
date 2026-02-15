/* 
 *  "Ping-Pong" autopilot using 1x1 rangefinder
 *
 *  Copyright (C) 2026 Simon D. Levy
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

// Hackflight
#include <simulator/dynamics.hpp>
#include <autopilot/rangefinder.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

#include "autopilot.hpp"

class SingleBeamRangefinder {

    friend class PingPongAutopilot;

    private:

        simsens::Rangefinder * rangefinder;
        
        int distance_mm;

        void init(simsens::Robot & robot, const string name)
        {
            rangefinder = robot.rangefinders[name];
        }

        void read(simsens::World & world, const hf::Dynamics::state_t & state)
        {
            rangefinder->read(
                    {state.x, state.y, state.z, state.phi, state.theta, state.psi},
                    world, &distance_mm);
         }

};

class PingPongAutopilot : public Autopilot {

    private:

        SingleBeamRangefinder _rangefinderForward;
        SingleBeamRangefinder _rangefinderBackward;

    public:

        void init(simsens::Robot & robot)
        {
            _rangefinderForward.init(robot, "VL53L1-forward");
            _rangefinderBackward.init(robot, "VL53L1-backward");
        }

        void getSetpoint(const hf::Dynamics::state_t state, hf::demands_t & setpoint) 
        {
            static constexpr int WALL_PROXIMITY_MM = 200;
            static constexpr float DY_ZERO = 1e-6;
            static constexpr float SPEED = 0.5;

            setpoint.pitch =

                // On startup, move forward (arbitrary)
                fabs(state.dy) < DY_ZERO ? +SPEED :

                // Close to forward wall, go backward
                _rangefinderForward.distance_mm < WALL_PROXIMITY_MM ? -SPEED :

                // Close to backward wall, go forward
                _rangefinderBackward.distance_mm < WALL_PROXIMITY_MM ? +SPEED :

                // Otherwise, continue in same direction
                state.dy > 0 ? -SPEED : +SPEED;
        }

        void readSensors(simsens::World & world,
                const hf::Dynamics::state_t & state, FILE * logfile)
        {
            (void)logfile;

            _rangefinderForward.read(world, state);
            _rangefinderBackward.read(world, state);
        }
};
