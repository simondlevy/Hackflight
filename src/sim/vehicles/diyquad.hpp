/* 
 * Vehicle parameters for simulated DIY quadcopter
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <sim/dynamics.hpp>

namespace hf {

    static constexpr Dynamics::vehicle_params_t VPARAMS = {

        1.0e-1, // mass [kg]
        5.0e-2, // arm length L [m]

        3.6e-5, // force coefficient B [F=b*w^2]
        7.0e-6, // drag coefficient D [T=d*w^2]
        2.0e-5  // I [kg*m^2]   // pitch, roll
    };

    // Might be derivable from above
    static constexpr float MOTOR_HOVER = 74.565; // rad/sec
}
