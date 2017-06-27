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
// PID config: YOU MUST SET THESE VALUES FOR EACH MODEL
//=========================================================================

struct PidConfig {

    // Level (accelerometer)
    float levelP;

    // Rate (gyro): P must be positive
    float ratePitchrollP;
    float ratePitchrollI;
    float ratePitchrollD;

    // Yaw: P must be positive
    float yawP;
    float yawI;

    // Trim for a particular vehicle: roll, pitch, yaw
    int16_t softwareTrim[3] = {0, 0, 0};
};

//=========================================================================
// Loop time config
//=========================================================================

struct LoopConfig {

    uint32_t imuLoopMicro       = 3500;
    uint32_t angleCheckMilli    = 500;
    uint32_t rcLoopMilli        = 10;
    uint32_t altHoldLoopMilli   = 25;
};

//=========================================================================
// IMU config
//=========================================================================

struct ImuConfig {

    float    maxAngleInclination    = 50.f; 
    float    maxArmingAngle         = 25.f;         
};

//=========================================================================
// Barometer config
//=========================================================================

struct BarometerConfig {

    uint8_t  calibrationSeconds = 8;
    float    noiseLpf           = 0.5f;

    static const uint8_t HISTORY_SIZE = 48;
};

//=========================================================================
// Accelerometer config (for atltitude hold)
//=========================================================================

struct AccelerometerConfig {

    uint16_t oneG        = 4096; // Raw accelerometer value when board is level
    float    lpfCutoff   = 5.0f;
    uint8_t  lpfFactor   = 4;
    int32_t  deadband    = 40;
};

//=========================================================================
// Altitude-hold config
//=========================================================================

struct AltitudeConfig {

    // PID
    float    pidAltP                = 0.40f;
    float    pidVelP                = 3.75f;
    float    pidVelI                = 0.005f;
    float    pidVelD                = 0.002f;

    // Barometer
    BarometerConfig baro;

    // Acceleromter
    AccelerometerConfig accel;

    // Complementry filter for accel/baro
    float    cfAlt                  = 0.965f;
    float    cfVel                  = 0.985f;

    // Fused
    uint8_t  maxTiltAngle           = 80;
    uint8_t  throttleNeutral        = 40;
    uint16_t throttleMin            = 1150;
    uint16_t throttleMax            = 1850;
};

//=========================================================================
// PWM config
//=========================================================================

struct PwmConfig {

    uint16_t min = 1000;
    uint16_t max = 2000;
};

//=========================================================================
// RC config
//=========================================================================

struct RcConfig {

    uint16_t mincheck   = 1100;
    uint16_t maxcheck   = 1900;
    int16_t expo8       = 65;
    int16_t rate8       = 90;
    int8_t  thrMid8     = 50;
    int32_t thrExpo8    = 0;
};

//=========================================================================
// initialization config
//=========================================================================

struct InitConfig {

    uint32_t delayMilli    = 100;
    uint32_t ledFlashMilli = 1000;
    uint32_t ledFlashCount = 20;
};

//=========================================================================
// all config
//=========================================================================

struct Config {
    LoopConfig     loop;
    ImuConfig      imu;
    AltitudeConfig altitude;
    RcConfig       rc;
    PidConfig      pid;
    PwmConfig      pwm;
    InitConfig     init;
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
// static constants, to avoid dynamic memory allocation
//=========================================================================

static const uint8_t CONFIG_RC_CHANS                = 8;
static const uint8_t CONFIG_PITCH_LOOKUP_LENGTH     = 7;
static const uint8_t CONFIG_THROTTLE_LOOKUP_LENGTH  = 12;

//=========================================================================
// STM32 reboot support
//=========================================================================

#define CONFIG_REBOOT_CHARACTER 'R'

} // namespace
