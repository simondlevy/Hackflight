#include <stdio.h>

// Hackflight
#include <datatypes.h>
#include <simulator/dynamics.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE\n", argv[0]);
        return 1;
    }

    simsens::WorldParser worldParser = {};
    worldParser.parse(argv[1], "DiyQuad {");

    simsens::RobotParser robotParser = {};
    robotParser.parse(argv[2]);

    const auto pose = worldParser.robotPose;

    printf("x=%+3.3f y=%+3.3f z=%+3.3f phi=%+3.3f theta=%+3.3f psi=%+3.3f\n",
            pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);

    return 0;
}
