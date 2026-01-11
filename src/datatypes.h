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

static constexpr float PID_FAST_FREQ = 500;  // 1024 Plank 
static constexpr float PID_SLOW_FREQ = 100;

typedef enum {

    MODE_IDLE,
    MODE_ARMED,
    MODE_HOVERING,
    MODE_AUTONOMOUS,
    MODE_LANDING,
    MODE_PANIC

} flightMode_t;

typedef enum {
    STATUS_IDLE,
    STATUS_ARMED,
    STATUS_HOVERING,
    STATUS_LANDING,
    STATUS_LOST_CONTACT

} status_t;


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

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

struct vec3_s {
    uint32_t timestamp; // Timestamp when the data was computed
    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;

typedef struct quaternion_s {
    union {
        struct {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

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

typedef struct flowMeasurement_s {
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

typedef struct tofMeasurement_s {
    uint32_t timestamp;
    float distance;
    float stdDev;
} tofMeasurement_t;

typedef struct
{
    Axis3f gyro; // deg/s, for legacy reasons
} gyroscopeMeasurement_t;

typedef struct
{
    Axis3f acc; // Gs, for legacy reasons
} accelerationMeasurement_t;

typedef struct {

    uint32_t timestamp;
    bool armed;
    bool hovering;
    demands_t demands;

} setpoint_t;

