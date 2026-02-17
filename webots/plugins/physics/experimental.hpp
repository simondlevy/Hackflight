/* 
 * Webots experimental custom physics plugin support for Hackflight
 *
 *  Copyright (C) 2025 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <stdio.h>

#include <simsensors/src/types.h>

class Experimental {

    public:

        static void write_to_log(
                FILE * logfile,
                const simsens::pose_t & pose,
                const int * rangefinder_distances,
                const int n_distances)
        {       
            fprintf(logfile,
                    "%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f",
                    pose.x, pose.y, pose.z, pose.phi, pose.theta, pose.psi);

            for (int k=0; k<n_distances; ++k) {
                fprintf(logfile, ",%d", rangefinder_distances[k]);
            }

            fprintf(logfile, "\n");


        }
};
