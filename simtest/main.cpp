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

// C
#include <stdio.h>

// Hackflight
#include <autopilot/twoexit.hpp>
#include <datatypes.h>
#include <simulator/simulator.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static constexpr float MAX_TIME_SEC = 10;
static constexpr float TAKEOFF_TIME_SEC = 2;

static const char * LOGNAME = "log.csv";

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE\n", argv[0]);
        return 1;
    }

    auto  * logfile = fopen(LOGNAME, "w");
    if (!logfile) {
        fprintf(stderr, "Unable to open file %s for wirting\n", LOGNAME);
        exit(1);
    }

    const auto robot_path = argv[1];
    simsens::Robot robot = {};
    simsens::RobotParser::parse(argv[1], robot);

    const auto world_path = argv[2];
    static simsens::World world = {};
    simsens::WorldParser::parse(world_path, world, robot_path);

    auto rangefinder = robot.rangefinders["VL53L5-forward"];

    const auto pose = world.getRobotPose();

    hf::Simulator simulator = {};

    const auto rate = hf::TwoExitAutopilot::FRAME_RATE_HZ;

    simulator.init({pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
           rate);

    int _rangefinder_distances_mm[1000] = {}; 

    hf::TwoExitAutopilot autopilot = {};

    for (int frame=0;
            frame<MAX_TIME_SEC * rate;
            ++frame) {

        const auto mode =
            frame < TAKEOFF_TIME_SEC * rate ?
            hf::MODE_HOVERING :
            hf::MODE_AUTONOMOUS;

        hf::demands_t setpoint = {};

        if (autopilot.run(frame, _rangefinder_distances_mm, setpoint)) {
            printf("succeeded\n");
            break;
        }

        const auto state = simulator.step(mode, setpoint);

        autopilot.writeToLog(logfile, state,
                _rangefinder_distances_mm, rangefinder->width);

        if (world.collided({state.x, state.y, state.z})) {
            printf("collided\n");
            break;
        }

        rangefinder->read(
                {state.x, state.y, state.z, state.phi, state.theta, state.psi},
                world, _rangefinder_distances_mm);
    }

    fclose(logfile);

    return 0;
}
