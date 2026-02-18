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
#include <stdlib.h>

// Hackflight
#include <autopilots/pingpong.hpp>
#include <simulator/simulator.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static constexpr float MAX_TIME_SEC = 10;
static constexpr float TAKEOFF_TIME_SEC = 2;
static const char * LOGNAME = "logpong.csv";
static const char * ROBOT_PATH = "../webots/protos/DiyQuad.proto";
static const char * WORLD_PATH = "../webots/worlds/pingpong.wbt";
static constexpr float FRAME_RATE_HZ = 32;

int main()
{
    auto  * logfile = fopen(LOGNAME, "w");
    if (!logfile) {
        fprintf(stderr, "Unable to open file %s for writing\n", LOGNAME);
        return 1;
    }

    fprintf(logfile, "pingpong\n");

    simsens::Robot robot = {};
    if (!simsens::RobotParser::parse(ROBOT_PATH, robot)) {
        return 1;
    }

    static simsens::World world = {};
    if (!simsens::WorldParser::parse(WORLD_PATH, world, ROBOT_PATH)) {
        return 1;
    }

    hf::PingPongAutopilot autopilot = {};

    const auto pose = world.getRobotPose();

    hf::Simulator simulator = {};

    simulator.init({pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi},
            FRAME_RATE_HZ);

    for (int frame=0; frame<MAX_TIME_SEC * FRAME_RATE_HZ; ++frame) {

        const auto mode =
            frame < TAKEOFF_TIME_SEC * FRAME_RATE_HZ ?
            hf::MODE_HOVERING :
            hf::MODE_AUTONOMOUS;

        // Start with neutral setpoint
        hf::demands_t setpoint = {};

        // Get current vehicle state based on setpoint
        const auto state = simulator.step(mode, setpoint);

        // Replace neutral setpoint with setpoint from autopilot if available
        if (mode == hf::MODE_AUTONOMOUS) {
            autopilot.getSetpoint(state.dy, setpoint);
        }


        // Get net state based on new setpoint
        const auto newstate = simulator.step(mode, setpoint);

        // Grab rangefinder readings for next iteration
        autopilot.readSensors(robot, world,
                {newstate.x, newstate.y, newstate.z,
                newstate.phi, newstate.theta, newstate.psi});

        fprintf(logfile,
                "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%d,%d\n",
                state.x, state.y, state.z, state.phi, state.theta, state.psi,
                autopilot.distance_forward_mm, 
                autopilot.distance_backward_mm);

        if (world.collided({state.x, state.y, state.z})) {
            printf("collided\n");
            break;
        }
    }

    fclose(logfile);

    printf("Wrote %s\n", LOGNAME);

    return 0;
}
