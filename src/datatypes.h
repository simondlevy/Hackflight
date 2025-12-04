/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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
    MODE_LOST_CONTACT

} flightMode_t;

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

// From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
// We use ENU coordinates based on 
// https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
// Position in meters, velocity in meters/second, angles in degrees,
// angular velocity in degrees/second.
typedef struct {

    float x;       // positive forward
    float dx;      // positive forward
    float y;       // positive leftward
    float dy;      // positive leftward
    float z;       // positive upward
    float dz;      // positive upward
    float phi;     // positive roll right
    float dphi;    // positive roll right
    float theta;   // positive nose up
    float dtheta;  // positive nose up (opposite of gyro Y)
    float psi;     // positive nose left
    float dpsi;    // positive nose left

} vehicleState_t;

// Limits for state vector, to support conversion to/from eight-bit rep
static const float STATE_DXY_MAX = 2;
static const float STATE_Z_MIN = 0;
static const float STATE_Z_MAX = 3;
static const float STATE_DZ_MAX = 1;
static const float STATE_PHITHETA_MAX = 30;
static const float STATE_DPHITHETA_MAX = 250;
static const float STATE_PSI_MAX = 180;
static const float STATE_DPSI_MAX = 250;

typedef struct {
    uint32_t timestamp;
    union {
        struct {
            float dpixelx;  // Accumulated pixel count x
            float dpixely;  // Accumulated pixel count y
        };
        float dpixel[2];  // Accumulated pixel count
    };
    float stdDevX;      // Measurement standard deviation
    float stdDevY;      // Measurement standard deviation
    float dt;           // Time during which pixels were accumulated

} flowMeasurement_t;

typedef struct {
    float distance;
    float stdDev;

} tofMeasurement_t;

typedef struct {
    axis3_t gyro; // deg/s, for legacy reasons

} gyroscopeMeasurement_t;

typedef struct {
    axis3_t acc; // Gs, for legacy reasons

} accelerationMeasurement_t;

typedef struct {
    uint32_t timestamp;
    bool armed;
    bool hovering;
    demands_t demands;

} command_t;
