/*
    Vehicle constants for Bitcraze Crazyflie

    Copyright (C) 2024 Simon D. Levy

    MIT License
*/

#pragma once

#include <dynamics.hpp>

vehicle_params_t crazyflie_vehicle_params = {

    // Estimated
    4.0e-5, // force constant B [F=b*w^2]
    2.0e-2, // torque constant D [T=d*w^2]

    // These agree with values in .proto file
    0.050,  // mass M [kg]
    0.031,  // arm length L [m]

    // Estimated
    2,      // Ix [kg*m^2]
    2,      // Iy [kg*m^2]
    3,      // Iz [kg*m^2]
    3.8e-3  // Jr prop inertial [kg*m^2]
};
