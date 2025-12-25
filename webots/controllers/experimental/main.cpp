/* 
   C++ flight simulator support for Hackflight with custom physics plugin

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
#include <unistd.h>
using namespace std;

// Hackflight
#include <setpoint/multiranger.hpp>

// Misc.
#include "../support.hpp"

static bool flight_mode_hovering(const flightMode_t mode)
{
    return mode == MODE_HOVERING;
}

int main(int argc, char ** argv) 
{
    (void)argc;

    const std::string worldname =  argv[1];

    Support support = {};

    support.begin();

    flightMode_t flightMode = MODE_IDLE;

    FILE * logfp = fopen("/home/levys/Desktop/hackflight/webots/controllers/"
            "experimental/groundtruth.csv", "w");

    const char * mode_names[6] = {
        "IDLE", "ARMED", "HOVERING", "AUTONOMOUS", "LANDING", "PANIC"
    };

    while (true) {

        Simulator::info_t siminfo = {};

        if (!support.beginStep(flight_mode_hovering, flightMode, siminfo)) {
            break;
        }

        printf("mode=%s\n", mode_names[flightMode]);

        strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
        strcpy(siminfo.worldname, worldname.c_str());

        int16_t ranger_distance_mm[1000] = {}; // arbitrary max size

        support.readRanger(ranger_distance_mm);

        //rv.show(ranger_distance_mm, LIDAR_DISPLAY_SCALEUP);

        //MultiRanger::getSetpoint(8, 8, ranger_distance_mm, siminfo.setpoint);
        fprintf(logfp, "%d\n", ranger_distance_mm[0]);

        support.endStep(siminfo, flightMode);

    }

    return support.end();
}
