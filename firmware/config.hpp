/*
   config.hpp : configuration values (PID etc.) for a specific vehicle

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>

namespace hf {

//=========================================================================
// PID config
//=========================================================================

struct PidConfig {

    // Level (accelerometer)
    uint8_t levelP;
    uint8_t levelI;

    // Rate (gyro): P must be positive
    uint8_t ratePitchrollP;
    uint8_t ratePitchrollI;
    uint8_t ratePitchrollD;

    // Yaw: P must be positive
    uint8_t yawP;
    uint8_t yawI;
};

//=========================================================================
// Loop time config
//=========================================================================

struct LoopConfig {

    uint32_t imuLoopMicro;
    uint32_t calibratingGyroMilli;
    uint32_t calibratingAccelMilli;
    uint32_t accelCalibrationPeriodMilli;
    uint32_t rcLoopMilli;
};

//=========================================================================
// IMU config
//=========================================================================

// NB: angles are in tenths of a degree

struct ImuConfig {

    uint16_t acc1G;
    float    accelLpfFactor;
    int32_t  accelZDeadband;
    int32_t  accelXyDeadband;
    float    accelzLpfCutoff;
    float    gyroCmpfFactor;
    float    gyroScale; 
    uint16_t maxAngleInclination;
    float    moronThreshold;  // variance in motion that triggers recalibration
    uint16_t smallAngle; 
};


//=========================================================================
// PWM config
//=========================================================================

struct PwmConfig {

    uint16_t min;
    uint16_t max;
};

//=========================================================================
// RC config
//=========================================================================

struct RcConfig {

    uint16_t mincheck;
    uint16_t maxcheck;
    int16_t expo8;
    int16_t rate8;
    int16_t thrMid8;
    int32_t thrExpo8;
};

// Keep these static const for now, to avoid dynamic memory allocation
static const uint8_t CONFIG_RC_CHANS                = 8;
static const uint8_t CONFIG_PITCH_LOOKUP_LENGTH     = 7;
static const uint8_t CONFIG_THROTTLE_LOOKUP_LENGTH  = 12;

//=========================================================================
// all config
//=========================================================================

struct Config {
    LoopConfig loop;
    ImuConfig imu;
    RcConfig rc;
    PidConfig pid;
    PwmConfig pwm;
    uint32_t initDelayMs = 100;
    uint32_t ledFlashCountOnStartup = 20;
};

//=========================================================================
// shared constants
//=========================================================================

enum {
    DEMAND_ROLL = 0,
    DEMAND_PITCH,
    DEMAND_YAW,
    DEMAND_THROTTLE,
    DEMAND_AUX1,
    DEMAND_AUX2,
    DEMAND_AUX3,
    DEMAND_AUX4
};

//=========================================================================
// STM32 reboot support
//=========================================================================

#define CONFIG_REBOOT_CHARACTER 'R'

} // namespace
