#include <stdio.h>

// Hackflight
#include <datatypes.h>
#include <simulator/simulator.hpp>
#include <pidcontrol.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static const uint32_t STEPS = 1000;
static const float FRAMERATE = 32;

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

    PidControl pidControl = {};

    Simulator simulator = {};

    simulator.init(&pidControl);

    simulator.setPoseAndFramerate(
            {pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi}, 
            FRAMERATE);

    FILE * logfp = fopen("log.csv", "w");

    for (uint32_t k=0; k<STEPS; ++k) {

        // const auto pose = simulator.step(MODE_IDLE, setpoint);

        fprintf(logfp, "%f,%f,%f,%f,%f,%f\n",
                pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);
    }

    fclose(logfp);

    return 0;
}
