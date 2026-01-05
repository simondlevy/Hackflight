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

#include <stdio.h>

#include <datatypes.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

/*
   while (wb_robot_step(TIME_STEP) != -1) {
// this is done repeatedly
const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
printf("MY_ROBOT is at position: %g %g %g\n", values[0], values[1], values[2]);
}

 */

static constexpr char ROBOT_NAME[] = "diyquad";

int main(int argc, char ** argv) 
{
    (void)argc;

    const char * logfilename =  argv[1];

    FILE * logfp = fopen(logfilename, "r");

    if (!logfp) {
        fprintf(stderr, "Unable to open file %s for input\n", logfilename);
        return 1;
    }

    wb_robot_init();

    const double timestep = wb_robot_get_basic_time_step();

    WbNodeRef robot_node = wb_supervisor_node_get_from_def(ROBOT_NAME);

    WbFieldRef trans_field =
        wb_supervisor_node_get_field(robot_node, "translation");

    while (true) {

        char line[1000] = {};

        if (!fgets(line, sizeof(line), logfp)) {
            break;
        }

        pose_t pose = {};

        sscanf(line, "%f,%f,%f,%f,%f,%f", 
                &pose.x, &pose.y, &pose.z,
                &pose.phi, &pose.theta, &pose.psi);

        if (wb_robot_step(timestep) == -1) {
            break;
        }

        const double xyz[3] = {pose.x, pose.y, pose.z};

        wb_supervisor_field_set_sf_vec3f(trans_field, xyz);

    }

    printf("done\n");

    wb_robot_cleanup();

    return 0;
}
