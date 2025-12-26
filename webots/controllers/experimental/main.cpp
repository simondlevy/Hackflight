/* 
   C++ flight simulator outerLoop for Hackflight with custom physics plugin

   Copyright (C) 2025 Simon D. Levy

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

// C/C++
#include <string.h>
#include <unistd.h>
using namespace std;

// Hackflight
#include <setpoint/multiranger.hpp>
#include <simulator/outer.hpp>

int main(int argc, char ** argv) 
{
    (void)argc;

    const std::string worldname =  argv[1];

    SimOuterLoop outerLoop = {};

    outerLoop.begin();

    FILE * logfp = fopen("/home/levys/Desktop/hackflight/webots/controllers/"
            "experimental/groundtruth.csv", "w");

    while (true) {

        siminfo_t siminfo = {};

        if (!outerLoop.beginStep(outerLoop.getFlightMode() == MODE_AUTONOMOUS,
                    siminfo)) {
            break;
        }

        strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
        strcpy(siminfo.worldname, worldname.c_str());

        int16_t ranger_distance_mm[1000] = {}; // arbitrary max size

        outerLoop.readRanger(ranger_distance_mm);

        //rv.show(ranger_distance_mm, LIDAR_DISPLAY_SCALEUP);

        //MultiRanger::getSetpoint(8, 8, ranger_distance_mm, siminfo.setpoint);
        fprintf(logfp, "%d\n", ranger_distance_mm[0]);

        outerLoop.endStep(siminfo);

    }

    return outerLoop.end();
}
