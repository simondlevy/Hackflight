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

/**
 * State vector as defined in Bouabdallah, Murrieri, and Siegwart (2004), with the following
 * modifications:
 *    
 * Linear velocities dX/dt, dY/dt are in body coordinates
 */ 
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

/**
 * Axis demands
 */
typedef struct {

    float thrust;
    float roll;
    float pitch;
    float yaw;

} demands_t;

typedef struct {

    float x;
    float y;

} axis2_t;

typedef struct {

    float x;
    float y;
    float z;

} axis3_t;

typedef struct {

    float w;
    float x;
    float y;
    float z;

} axis4_t;

enum {

    STATUS_READY,
    STATUS_ARMED,
    STATUS_FAILSAFE

};
