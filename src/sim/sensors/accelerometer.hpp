/*
 *   Accelerometer simulator
 *
 *   Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, in version 3.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <iostream>
#include <random>
#include <vector>

#include <hackflight.hpp>
#include <utils.hpp>

namespace hf {

    class Accelerometer {

        public:

            static axis3_t read(const Dynamics & d, const float noise=1e-6)
            {
                const auto s = Utils::G2MSS;

                std::random_device rd{};
                std::mt19937 gen{rd()};
                std::normal_distribution<float> dist{0, noise}; 

                const auto accel =
                    d._airborne ? 
                    axis3_t { d.x7 / s, d.x9 / s, (d._dx6 + d._g) / s }  :
                    axis3_t {0, 0, +1};

                return axis3_t {
                    accel.x + dist(gen),
                    accel.y + dist(gen),
                    accel.z + dist(gen)
                };
            }
    };
}
