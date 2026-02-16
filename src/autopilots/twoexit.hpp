/**
 * "Two-exit" autopilot using 1x8 rangefinder
 *
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Hackflight
#include <datatypes.h>

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

namespace hf {

    class TwoExitAutopilot {

        public:

            static constexpr float FRAME_RATE_HZ = 32;

            int rangefinder_distances_mm[8];

            simsens::Rangefinder * get_rangefinder(simsens::Robot & robot)
            {
                return robot.rangefinders["VL53L5-forward"];
            }

            bool getSetpoint(const int frame, demands_t & setpoint)
            {
                static constexpr float TRAVEL_AFTER_CLEAR_SEC = 1;

                const int * d = rangefinder_distances_mm;

                // Look for clear (infinity reading) in center of 1x8 readings
                const bool center_is_clear = d[3] == -1 && d[4] == -1;

                // If clear, pitch forward
                setpoint.pitch = center_is_clear ? 0.4 : 0;

                // Otherwise, yaw rightward
                setpoint.yaw = center_is_clear ? 0 : 0.2;

                // We're not done until all readings are infinity
                for (int i=0; i<8; ++i) {
                    if (d[i] != -1) {
                        return false;
                    }
                }

                static int _cleared_at_frame;

                if (_cleared_at_frame == 0) {
                    _cleared_at_frame = frame;
                }

                // Travel a bit after exiting
                else if ((frame - _cleared_at_frame)/FRAME_RATE_HZ >
                        TRAVEL_AFTER_CLEAR_SEC) {
                    return true;
                }

                return false;
            }        

            void readSensors(
                    simsens::Robot & robot,
                    simsens::World & world,
                    const simsens::pose_t & pose,
                    FILE * logfile)
            {
                get_rangefinder(robot)->read(pose, world, rangefinder_distances_mm);

                const auto d = rangefinder_distances_mm;

                fprintf(logfile,
                        "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f," 
                        "%d,%d,%d,%d,%d,%d,%d,%d\n",
                        pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi,
                        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
            }
    };
}
