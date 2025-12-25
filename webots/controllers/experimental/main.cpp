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

static bool step(const string worldname, Support & support, FILE * logfp)
{
    static flightMode_t _flightMode;

    Simulator::info_t siminfo = {};

    if (!support.beginStep(flight_mode_hovering, _flightMode, siminfo)) {
        return false;
    }

    strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
    strcpy(siminfo.worldname, worldname.c_str());

    int16_t ranger_distance_mm[1000] = {}; // arbitrary max size

    support.readRanger(ranger_distance_mm);

    //rv.show(ranger_distance_mm, LIDAR_DISPLAY_SCALEUP);

    //MultiRanger::getSetpoint(8, 8, ranger_distance_mm, siminfo.setpoint);
    fprintf(logfp, "%d\n", ranger_distance_mm[0]);

    support.endStep(siminfo, _flightMode);

    return true;
}

int main(int argc, char ** argv) 
{
    (void)argc;

    Support support = {};

    support.begin();

    const std::string worldname =  argv[1];

    FILE * logfp = fopen("/home/levys/Desktop/hackflight/webots/controllers/"
            "experimental/groundtruth.csv", "w");

    while (true) {

        if (!step(worldname, support, logfp)) {
            break;
        }
    }

    return support.end();
}
