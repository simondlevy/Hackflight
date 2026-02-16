/* 
 *  Two-exit autopilot using 1x8 rangefinder
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
#include <autopilots/twoexit.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

#include "autopilot.hpp"

class TwoExitAutopilot : public Autopilot {

    private:

        static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

        hf::TwoExitAutopilot _helper;

    public:

        void getSetpoint(hf::Dynamics::state_t state, hf::demands_t & setpoint) 
        {
            (void)state;

            static int _frame;

            _helper.getSetpoint(_frame++, setpoint);
         }

        void readSensors(
                simsens::Robot & robot,
                simsens::World & world,
                const hf::Dynamics::state_t & state,
                FILE * logfile)
        {
            // Get simulated rangefinder distances based on new pose
            _helper.readSensors(robot, world,
                    {state.x, state.y, state.z,
                     state.phi, state.theta, state.psi},
                     logfile);

            // Visualize rangefinder distances
            simsens::RangefinderVisualizer::show(
                    _helper.rangefinder_distances_mm,
                    _helper.get_rangefinder(robot)->min_distance_m,
                    _helper.get_rangefinder(robot)->max_distance_m,
                    8, 1, RANGEFINDER_DISPLAY_SCALEUP);
        }
};
