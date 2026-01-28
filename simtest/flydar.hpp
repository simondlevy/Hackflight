/**
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

// C/C++
#include <stdio.h>

// Hackflight
#include <autopilot/rangefinder.hpp>
#include <datatypes.h>
#include <simulator/simulator.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

class SimTest {

    private:

        static constexpr float MAX_TIME_SEC = 10;
        static constexpr float TAKEOFF_TIME_SEC = 2;
        static constexpr float FRAMERATE_HZ = 32;

    public:

        static void run(
                const char * robot_path,
                const char * world_path,
                FILE * logfile=nullptr)
        {

            simsens::Robot robot = {};
            simsens::RobotParser::parse(robot_path, robot);

            simsens::World world = {};
            simsens::WorldParser::parse(world_path, world, robot_path);

            simsens::Rangefinder rangefinder =
                simsens::Rangefinder(*robot.rangefinders[0]);

            const auto pose = world.getRobotPose();

            Simulator simulator = {};

            simulator.init(
                    {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
                    FRAMERATE_HZ);

            int rangefinder_distances_mm[1000] = {}; // arbitrary max size

            for (uint32_t t=0; t<MAX_TIME_SEC*FRAMERATE_HZ; ++t) {

                const auto mode = t < TAKEOFF_TIME_SEC*FRAMERATE_HZ ? MODE_HOVERING :
                    MODE_AUTONOMOUS;

                demands_t setpoint = {};

                RangefinderSetpoint::run(rangefinder_distances_mm, setpoint);

                const auto pose = simulator.step(mode, setpoint);

                if (world.collided({pose.x, pose.y, pose.x})) {
                    printf("collision!\n");
                    break;
                }

                // Get simulated rangefinder distances
                rangefinder.read(
                        {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                        world, rangefinder_distances_mm);

                // Dump everything to logfile
                if (logfile) {
                    fprintf(logfile, "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f", 
                            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
                    for (int k=0; k<rangefinder.getWidth(); ++k) {
                        fprintf(logfile, ",%d", rangefinder_distances_mm[k]);
                    }
                    fprintf(logfile, "\n");
                }
            }
        }

};
