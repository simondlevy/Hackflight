/**
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>

// C/C++
#include <stdio.h>

// Hackflight
#include <autopilot/rangefinder.hpp>
#include <datatypes.h>
#include <simulator/simulator.hpp>
#include <vehicles/diyquad.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

#include "flydar.hpp"

static constexpr float TAKEOFF_TIME_SEC = 2;
static constexpr float FRAME_RATE_HZ = 32;

static const char * LOGNAME = "log.csv";

int main(int argc, char ** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s WORLDFILE ROBOTFILE\n", argv[0]);
        return 1;
    }

    auto  * logfile = fopen(LOGNAME, "w");
    if (!logfile) {
        fprintf(stderr, "Unable to open file %s for wirting\n", LOGNAME);
        exit(1);
    }

    Flydar flydar = Flydar(argv[1], argv[2]);

    for (int frame=0; frame<flydar.maxframes(); ++frame) {

        if (flydar.step(frame, logfile)) {
            break;
        }
    }

    fclose(logfile);

    return 0;
}
