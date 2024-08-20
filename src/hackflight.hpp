/*
   Hackflight main header with datatypes

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

#pragma once

#include <stdint.h>

namespace hf {

    typedef struct {

        float x;
        float y;
        float z;

    } axis3_t;

    // From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004)
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

    typedef struct {

        uint32_t c1;
        uint32_t c2;
        uint32_t c3;
        uint32_t c4;
        uint32_t c5;
        uint32_t c6;

    } channels_t;

    typedef struct {

        float m1;
        float m2;
        float m3;
        float m4;

    } quad_motors_t;

    typedef struct {

        float w;
        float x;
        float y;
        float z;

    } quat_t;

}
