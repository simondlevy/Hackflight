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
        static constexpr float TRAVEL_AFTER_CLEAR_SEC = 1;
        static constexpr float FRAME_RATE_HZ = 32;

        static bool clear(const int * rangefinder_distances, const int size)
        {
            for (int i=0; i<size; ++i) {
                if (rangefinder_distances[i] != -1) {
                    return false;
                }
            }

            return true;
        }

        static bool cleared_room(
                const int frame, 
                const int * rangefinder_distances_mm, 
                const int rangefinder_size)
        {
            static int _cleared_at_frame;

            if (_cleared_at_frame == 0 &&
                    clear(rangefinder_distances_mm, rangefinder_size)) {
                _cleared_at_frame = frame;
            }

            if (_cleared_at_frame > 0 &&
                    (frame - _cleared_at_frame)/FRAME_RATE_HZ > TRAVEL_AFTER_CLEAR_SEC) {
                return true;
            }

            return false;
        }
        
        static void write_to_log(
                FILE * logfile,
                const pose_t pose,
                const int * rangefinder_distances_mm,
                const int rangefinder_width)
        {
            fprintf(logfile, "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f", 
                    pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
            for (int k=0; k<rangefinder_width; ++k) {
                fprintf(logfile, ",%d", rangefinder_distances_mm[k]);
            }
            fprintf(logfile, "\n");
        }

    public:

        static bool run(
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
                    FRAME_RATE_HZ);

            int rangefinder_distances_mm[1000] = {}; // arbitrary max size

            for (uint32_t frame=0; frame<MAX_TIME_SEC*FRAME_RATE_HZ; ++frame) {

                const auto mode = frame < TAKEOFF_TIME_SEC*FRAME_RATE_HZ ? MODE_HOVERING :
                    MODE_AUTONOMOUS;

                demands_t setpoint = {};

                RangefinderSetpoint::run(rangefinder_distances_mm, setpoint);

                const auto pose = simulator.step(mode, setpoint);

                if (world.collided({pose.x, pose.y, pose.x})) {
                    return false;
                }

                if (cleared_room(frame, rangefinder_distances_mm, rangefinder.getWidth())) {
                    return true;
                }

                rangefinder.read(
                        {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                        world, rangefinder_distances_mm);

                if (logfile) {
                    write_to_log(logfile, pose,
                            rangefinder_distances_mm, rangefinder.getWidth());
                }
            }

            return false; // timed out
        }

};
