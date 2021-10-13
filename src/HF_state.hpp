/*
   Datatype declarations for vehicle state

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

#include "HF_utils.hpp"

namespace hf {

    typedef struct {

        float x;
        float dx;
        float y;
        float dy;
        float z;
        float dz;
        float phi;
        float dphi;
        float theta;
        float dtheta;
        float psi;
        float dpsi;

    } state_t;

} // namespace hf
