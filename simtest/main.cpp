#include <stdio.h>

// Hackflight
#include <datatypes.h>
#include <simulator/dynamics.hpp>
#include <pidcontrol.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static constexpr float DYNAMICS_FREQ = 1e5; // Hz

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

    Dynamics dynamics = Dynamics(VPARAMS, 1./DYNAMICS_FREQ);

    dynamics.setPose({pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi});

    PidControl pidControl;

    pidControl.init();

    FILE * logfp = fopen("log.csv", "w");

    fclose(logfp);

    return 0;
}
