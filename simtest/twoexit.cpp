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
#include <autopilots/twoexit.hpp>
#include <simulator/simulator.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static constexpr float MAX_TIME_SEC = 10;
static constexpr float TAKEOFF_TIME_SEC = 2;
static const char * LOGNAME = "logexit.csv";
static const char * ROBOT_PATH = "../webots/protos/DiyQuad.proto";
static const char * WORLD_PATH = "../webots/worlds/twoexit.wbt";

int main()
{
    auto  * logfile = fopen(LOGNAME, "w");
    if (!logfile) {
        fprintf(stderr, "Unable to open file %s for writing\n", LOGNAME);
        return 1;
    }

    fprintf(logfile, "twoexit\n");

    simsens::Robot robot = {};
    if (!simsens::RobotParser::parse(ROBOT_PATH, robot)) {
        return 1;
    }

    static simsens::World world = {};
    if (!simsens::WorldParser::parse(WORLD_PATH, world, ROBOT_PATH)) {
        return 1;
    }

    hf::TwoExitAutopilot autopilot = {};

    const auto pose = world.getRobotPose();

    hf::Simulator simulator = {};

    const auto rate = hf::TwoExitAutopilot::FRAME_RATE_HZ;

    simulator.init({pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
           rate);

    for (int frame=0; frame<MAX_TIME_SEC * rate; ++frame) {

        const auto mode =
            frame < TAKEOFF_TIME_SEC * rate ?
            hf::MODE_HOVERING :
            hf::MODE_AUTONOMOUS;

        hf::demands_t setpoint = {};

        if (autopilot.getSetpoint(frame, setpoint)) {
            printf("succeeded\n");
            break;
        }

        const auto state = simulator.step(mode, setpoint);

        autopilot.readSensor(robot, world,
                {state.x, state.y, state.z,
                state.phi, state.theta, state.psi});

        const auto d = autopilot.rangefinder_distances_mm;

        fprintf(logfile,
                "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f"
                ",%d,%d,%d,%d,%d,%d,%d,%d\n",
                state.x, state.y, state.z, state.phi, state.theta, state.psi,
                d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

        if (world.collided({state.x, state.y, state.z})) {
            printf("collided\n");
            break;
        }
    }

    fclose(logfile);

    printf("Wrote %s\n", LOGNAME);

    return 0;
}
