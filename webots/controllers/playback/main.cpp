/* 
   Hackflight simulator playback

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

// Hackflight
#include <datatypes.h>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

// Webots
#include <webots/supervisor.h>
#include "../motors.hpp"

static constexpr char ROBOT_NAME[] = "diyquad";

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

// https://www.euclideanspace.com/maths/geometry/rotations/conversions/
//   eulerToAngle/index.htm
static void euler_to_rotation(const double euler[3], double rotation[4])
{
    const double phi = euler[0];
    const double theta = euler[1];
    const double psi = euler[2];

    const double c1 = cos(theta/2);
    const double c2 = cos(psi/2);
    const double c3 = cos(phi/2);
    const double s1 = sin(theta/2);
    const double s2 = sin(psi/2);
    const double s3 = sin(phi/2);

    rotation[0] = s1*s2*c3 + c1*c2*s3;
    rotation[1] = s1*c2*c3 + c1*s2*s3;
    rotation[2] = phi==0 && theta==0 && psi==0 ? 1 : c1*s2*c3 - s1*c2*s3;
    rotation[3] = 2 * acos(c1*c2*c3 - s1*s2*s3);
}


int main(int argc, char ** argv) 
{
    (void)argc;

    const char * worldname =  argv[1];
    const char * poselogname =  argv[2];

    char path[1000];

    simsens::WorldParser worldParser = {};

    sprintf(path, "../../worlds/%s.wbt", worldname);
    worldParser.parse(path);

    simsens::RobotParser robotParser = {};
    robotParser.parse("../../protos/DiyQuad.proto");

    simsens::Rangefinder * rangefinder = robotParser.rangefinders[0];

    simsens::RangefinderVisualizer rangefinderVisualizer =
        simsens::RangefinderVisualizer(rangefinder);


    FILE * logfp = fopen(poselogname, "r");

    if (!logfp) {
        fprintf(stderr, "Unable to open file %s for input\n", poselogname);
        return 1;
    }

    wb_robot_init();

    const double timestep = wb_robot_get_basic_time_step();

    WbNodeRef robot_node = wb_supervisor_node_get_from_def(ROBOT_NAME);

    WbFieldRef translation_field =
        wb_supervisor_node_get_field(robot_node, "translation");

    WbFieldRef rotation_field =
        wb_supervisor_node_get_field(robot_node, "rotation");

    startMotors();

    while (true) {

        char line[1000] = {};

        if (!fgets(line, sizeof(line), logfp)) {
            break;
        }

        simsens::pose_t pose = {};

        int rangefinder_distances_mm[8] = {};
        int * d = rangefinder_distances_mm;

        if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d", 
                &pose.x, &pose.y, &pose.z,
                &pose.phi, &pose.theta, &pose.psi,
                &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7]) == 14) {

            rangefinderVisualizer.show(rangefinder_distances_mm,
                    RANGEFINDER_DISPLAY_SCALEUP);
        }

        if (wb_robot_step(timestep) == -1) {
            break;
        }

        // Negate y for leftward negative
        const double xyz[3] = {pose.x, -pose.y, pose.z};
        wb_supervisor_field_set_sf_vec3f(translation_field, xyz);

        // Negate psi for nose-left negative
        const double euler[3] = {pose.phi, pose.theta, -pose.psi};
        double rotation[4] = {};
        euler_to_rotation(euler, rotation);
        wb_supervisor_field_set_sf_rotation(rotation_field, rotation);
    }

    printf("done\n");

    wb_robot_cleanup();

    return 0;
}
