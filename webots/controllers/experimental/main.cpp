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

#if 0
static demands_t getAutonomousSetpoint(const int16_t * ranger_distances_mm)
{
    typedef enum {
        PHASE_START,
        PHASE_ROTATE,
        PHASE_FORWARD,
        PHASE_DONE
    } phase_e;

    static phase_e _phase;

    const auto d = ranger_distances_mm;

    float yaw = 0;

    switch(_phase) {

        case PHASE_START:
            _phase = PHASE_ROTATE;
            break;

        case PHASE_ROTATE:
            yaw = 0.5;
            break;

        case PHASE_FORWARD:
            break;

        case PHASE_DONE:
            break;

    }

    /*
    printf("d0=%d d1=%d d2=%d d3=%d d4=%d d5=%d d6=%d d7=%d | %d => %+3.3f\n",
            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
            d[3] == -1 && d[4] == -1, yaw); */

    return demands_t {0.5, 0, 0, yaw};
}
#endif

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

        static int16_t ranger_distances_mm[1000]; // arbitrary max size

        if (outerLoop.getFlightMode() == MODE_AUTONOMOUS) {

            demands_t setpoint = {0.5, 0, 0, 0.5};

            if (!outerLoop.beginStep(siminfo, &setpoint)) {
                break;
            }
            //getAutonomousSetpoint(ranger_distances_mm);
        }

        else {
            if (!outerLoop.beginStep(siminfo)) {
                break;
            }
        }

        outerLoop.readRanger(ranger_distances_mm);

        fprintf(logfp, "%d\n", ranger_distances_mm[0]);

        outerLoop.endStep(siminfo);
    }

    return outerLoop.end();
}
