/**
 * Network performing A - B
 * 
 * Copyright (C) 2025 James Plank, Simon D. Levy
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

#pragma once

#include <tennlab/framework.hpp>

class DifferenceNetwork {

    private:

        static const uint16_t VIZ_PORT = 8100;
        static const uint32_t VIZ_SEND_PERIOD = 50; // ticks

        const char * NETWORK_FILENAME =
            "/home/levys/Desktop/tennlab-networks/difference_risp_plank.txt";

        static constexpr double MAX_SPIKE_TIME = 1000;

};
