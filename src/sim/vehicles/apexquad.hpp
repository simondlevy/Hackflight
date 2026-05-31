/* 
 * Vehicle parameters for 2S quadcopter on 2inch APEX frame
 *
 *  Copyright (C) 2025 Simon D. Levy
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

#pragma once

#include <sim/dynamics.hpp>


// Approximate thrust RPM needed when in perfect hover
static constexpr float THRUST_BASE_RPM = 3.6e4;

static constexpr float THRUST_SCALE_RPM = 1e3;
static constexpr float PITCH_ROLL_SCALE_RPM = 1e6;
static constexpr float YAW_SCALE_RPM = 2e4;


static constexpr hf::Dynamics::vehicle_params_t VPARAMS = {

    // Actual values
    9.0e-2,  // mass [kg]
    5.0e-2,  // arm length L [m]

    // Estimated by using constants above: 
    //   B, D increase in proportion to mass;
    //   I decreases in proportion to mass;
    1.3e-8, // force coefficient B [F=b*w^2]
    1.9e-9, // drag coefficient D [T=d*w^2]
    4.1e-4  // I [kg*m^2]   // pitch, roll
};
