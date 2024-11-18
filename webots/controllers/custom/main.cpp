/* 
   C++ flight simulator support for Hackflight with custom physics plugin
   
   Copyright (C) 2024 Simon D. Levy

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

#include <webots/gps.h>
#include <webots/robot.h>

#include <sim/sim2.hpp>

static WbDeviceTag _makeSensor(
        const char * name, 
        const uint32_t timestep,
        void (*f)(WbDeviceTag tag, int sampling_period))
{
    auto sensor = wb_robot_get_device(name);
    f(sensor, timestep);
    return sensor;
}


int main() 
{
    hf::Simulator sim = {};

    sim.init();

    const auto timestep = wb_robot_get_basic_time_step();

    const auto dt = 1 / timestep;

    auto gps = _makeSensor("gps", timestep, wb_gps_enable);

    float yprev = 0;

    while (sim.step()) {

        const auto y = wb_gps_get_values(gps)[1];

        const auto dy = (y - yprev) / dt;

        yprev = y;
    }

    sim.close();

    return 0;
}
