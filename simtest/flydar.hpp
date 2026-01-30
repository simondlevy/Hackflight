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
#include <simulator/pose.h>
#include <simulator/simulator.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

class Flydar {

    private:

        static constexpr float TRAVEL_AFTER_CLEAR_SEC = 1;
        static constexpr float MAX_TIME_SEC = 10;
        static constexpr float TAKEOFF_TIME_SEC = 2;
        static constexpr float FRAME_RATE_HZ = 32;

        hf::Simulator _simulator;

        simsens::World _world;

        simsens::Rangefinder * _rangefinder;

        static bool succeeded(
                const int frame, 
                const int * rangefinder_distances_mm, 
                const int rangefinder_size)
        {
            for (int i=0; i<rangefinder_size; ++i) {
                if (rangefinder_distances_mm[i] != -1) {
                    return false;
                }
            }

            static int _cleared_at_frame;

            if (_cleared_at_frame == 0) {
                _cleared_at_frame = frame;
            }

            else if ((frame - _cleared_at_frame)/FRAME_RATE_HZ > TRAVEL_AFTER_CLEAR_SEC) {
                return true;
            }

            return false;
        }

        static void write_to_log(
                FILE * logfile,
                const hf::pose_t pose,
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

        Flydar(const char * robot_path, const char * world_path)
        {
            // run(robot_path, world_path, logfile);
            simsens::Robot robot = {};
            simsens::RobotParser::parse(robot_path, robot);

            simsens::WorldParser::parse(world_path, _world, robot_path);

            _rangefinder = robot.rangefinders[0];

            const auto pose = _world.getRobotPose();

            _simulator.init(
                    {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
                    FRAME_RATE_HZ);
        }

        int maxframes()
        {
            return MAX_TIME_SEC * FRAME_RATE_HZ;
        }

        bool step(const int frame, FILE * logfile=nullptr)
        {
            static int _rangefinder_distances_mm[1000]; 

            const auto mode = frame < TAKEOFF_TIME_SEC*FRAME_RATE_HZ ?
                hf::MODE_HOVERING :
                hf::MODE_AUTONOMOUS;

            hf::demands_t setpoint = {};

            hf::RangefinderSetpoint::run(_rangefinder_distances_mm, setpoint);

            const auto pose = _simulator.step(mode, setpoint);

            if (logfile) {
                write_to_log(logfile, pose,
                        _rangefinder_distances_mm, _rangefinder->getWidth());
            }

            if (_world.collided({pose.x, pose.y, pose.x})) {
                return true;
            }

            if (succeeded(frame, _rangefinder_distances_mm,
                        _rangefinder->getWidth())) {
                return true;
            }

            _rangefinder->read(
                    {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
                    _world, _rangefinder_distances_mm);

            return false;
        }
};
