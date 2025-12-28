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

#include <string.h>

#include <unistd.h>
using namespace std;

#include <simulator/outer.hpp>

static bool isinf(const int16_t d)
{
    return d == -1;
}

static demands_t getAutonomousSetpoint(const int16_t * ranger_distances_mm)
{
    const auto d = ranger_distances_mm;

    const bool perimeter_is_clear = isinf(d[0]) && isinf(d[1]) && isinf(d[2])
            && isinf(d[6]) && isinf(d[6]) && isinf(d[7]);

    const bool center_is_clear = isinf(d[3]) && isinf(d[4]);

    const float thrust = 0.5; // hold altitude
    const float roll = 0;
    const float pitch = 
        perimeter_is_clear ? 0 :
        center_is_clear ? 0.2 :
        0;
    const float yaw = 0;//center_is_clear ? 0 : 0.2;

    return demands_t {thrust, roll, pitch, yaw};
}

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
        strcpy(siminfo.path, getcwd(siminfo.path, sizeof(siminfo.path)));
        strcpy(siminfo.worldname, worldname.c_str());

        static int16_t rangefinder_distances_mm[1000]; // arbitrary max size

        bool okay = true;

        if (outerLoop.getFlightMode() == MODE_AUTONOMOUS) {

            demands_t setpoint =
                getAutonomousSetpoint(rangefinder_distances_mm);

            okay = outerLoop.beginStep(siminfo, &setpoint);
        }

        else {
            okay = outerLoop.beginStep(siminfo);
        }

        if (!okay) {
            break;
        }

        int rangefinder_width=0, rangefinder_height=0;
        outerLoop.readRangefinder(
                rangefinder_distances_mm,
                rangefinder_width,
                rangefinder_height);

        static bool _ready; // synch with plugin
        if (_ready) {
            for (int k=0; k<rangefinder_width; ++k) {
                fprintf(logfp, "%d%c", rangefinder_distances_mm[k],
                        (k==rangefinder_width-1)?'\n':',');
            }
            fflush(logfp);
        }
        _ready = true;

        outerLoop.endStep(siminfo);

    }

    return outerLoop.end();
}
