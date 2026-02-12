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

// Hackflight
#include <simulator/pose.h>
#include <autopilot/rangefinder.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

class TwoExit {

    private:

        static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

        simsens::Rangefinder * _rangefinder;

        std::map<string, simsens::Rangefinder *> _rangefinders;

        int _rangefinder_distances_mm[1000]; // arbitrary max size

    public:

        void init(simsens::Robot & robot)
        {
            _rangefinder = robot.rangefinders["VL53L5-forward"];
            _rangefinders = robot.rangefinders;
        }

        void getSetpoint(PhysicsPluginHelper::siminfo_t & siminfo)
        {
            static int _frame;

            hf::RangefinderSetpoint::runTwoExit(_frame++,
                    _rangefinder_distances_mm, siminfo.setpoint);
        }

        void readSensors(simsens::World & world, const hf::pose_t & pose,
                FILE * logfile)
        {
            // Get simulated rangefinder distances based on new pose
            _rangefinder->read(
                    simsens::pose_t {
                    pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                    world, _rangefinder_distances_mm);

            // Dump everything to logfile
            fprintf(logfile, "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f", 
                    pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
            for (int k=0; k<_rangefinder->width; ++k) {
                fprintf(logfile, ",%d", _rangefinder_distances_mm[k]);
            }
            fprintf(logfile, "\n");

            // Visualize rangefinder distances
            simsens::RangefinderVisualizer::show(
                    _rangefinder_distances_mm,
                    _rangefinder->min_distance_m,
                    _rangefinder->max_distance_m,
                    _rangefinder->width,
                    _rangefinder->height,
                    RANGEFINDER_DISPLAY_SCALEUP);

        }
};
