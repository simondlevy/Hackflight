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

#pragma once

#include <stdint.h>

typedef enum {

    MODE_IDLE,
    MODE_ARMED,
    MODE_HOVERING,
    MODE_AUTONOMOUS,
    MODE_LANDING,
    MODE_PANIC

} mode_e;

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

typedef struct {

    float thrust;  // positve upward
    float roll;    // positive roll right
    float pitch;   // positive nose down
    float yaw;     // positive nose right

} demands_t;

typedef void (*mixFun_t)(const demands_t & demands, float motorvals[]);

