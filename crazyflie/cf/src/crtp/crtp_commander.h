/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 */

#pragma once

#include <stdint.h>

#include <datatypes.h>

#include "crtp.h"

typedef enum mode_e {
    modeDisable = 0,
    modeAbs,
    modeVelocity
} stab_mode_t;

typedef struct vec3_s velocity_t;

typedef struct vec3_s acc_t;

typedef struct attitude_s {
    uint32_t timestamp;  // Timestamp when the data was computed
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct setpoint_s {

    uint32_t timestamp;

    attitude_t attitude;      // deg
    attitude_t attitudeRate;  // deg/s
    quaternion_t attitudeQuaternion;
    float thrust;
    point_t position;         // m
    velocity_t velocity;      // m/s
    acc_t acceleration;       // m/s^2
    bool velocity_body;       // velocity in body frame vs. world frame

    struct {
        stab_mode_t x;
        stab_mode_t y;
        stab_mode_t z;
        stab_mode_t roll;
        stab_mode_t pitch;
        stab_mode_t yaw;
        stab_mode_t quat;
    } mode;

} setpoint_t;


void crtpCommanderInit(void);

void crtpCommanderOpenLoopDecodeSetpoint(
        setpoint_t *setpoint, crtpPacket_t *pk);

void crtpCommanderGenericDecodeSetpoint(
        setpoint_t *setpoint, crtpPacket_t *pk);
