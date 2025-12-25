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

// Webots
#include <webots/range_finder.h>

// Misc.
#include "../support.hpp"

static WbDeviceTag _ranger;

static void readRanger(const int width, const int height,
        int16_t * distance_mm) 
{
    const float * image = wb_range_finder_get_range_image(_ranger);

    for (int x=0; x<width; ++x) {

        for (int y=0; y<height; ++y) {

            const float distance_m =
                wb_range_finder_image_get_depth(image, width, x, y);

            distance_mm[y*width+x] = isinf(distance_m) ? -1 :
                (int16_t)(1000 * distance_m);
        }
    }
}

static bool flight_mode_hovering(const flightMode_t mode)
{
    return mode == MODE_HOVERING;
}

static bool step(const string worldname, const setpointType_e setpointType,
        FILE * logfp)
{
    static flightMode_t _flightMode;

    Simulator::info_t siminfo = {};

    if (!beginStep(flight_mode_hovering, _flightMode, siminfo)) {
        return false;
    }

    strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
    strcpy(siminfo.worldname, worldname.c_str());

    int16_t ranger_distance_mm[1000] = {}; // arbitrary max size

    const int width = wb_range_finder_get_width(_ranger);
    const int height = wb_range_finder_get_height(_ranger);

    readRanger(width, height, ranger_distance_mm);

    //rv.show(ranger_distance_mm, LIDAR_DISPLAY_SCALEUP);

    if (setpointType == SETPOINT_LIDAR) {
        MultiRanger::getSetpoint(8, 8, ranger_distance_mm, siminfo.setpoint);
    }
    fprintf(logfp, "%d\n", ranger_distance_mm[0]);

    endStep(siminfo, _flightMode);

    return true;
}

int main(int argc, char ** argv) 
{
    (void)argc;

    begin();

    const std::string worldname =  argv[1];
    const std::string setpoint =  argv[2];

    _ranger = wb_robot_get_device("range-finder");
    wb_range_finder_enable(_ranger, _timestep);

    setpointType_e setpointType = SETPOINT_HUMAN;

    if (setpoint == "lidar") {
    }
    else if (setpoint == "human") {
    }
    else {
        printf("Unrecognized setpoint '%s'; defaulting to human\n",
                setpoint.c_str());
    }


    FILE * logfp = fopen("/home/levys/Desktop/hackflight/webots/controllers/"
            "experimental/groundtruth.csv", "w");

    while (true) {

        if (!step(worldname, setpointType, logfp)) {
            break;
        }
    }

    return end();
}
