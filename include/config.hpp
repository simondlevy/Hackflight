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
    uint8_t levelP;
    uint8_t levelI;

    // Rate (gyro): P must be positive
    uint8_t ratePitchrollP;
    uint8_t ratePitchrollI;
    uint8_t ratePitchrollD;

    // Yaw: P must be positive
    uint8_t yawP;
    uint8_t yawI;

    // Trim for a particular vehicle
    int16_t softwareTrim[3] = {0, 0, 0};
};

//=========================================================================
// Loop time config
//=========================================================================

struct LoopConfig {

    uint32_t angleCheckMilli	            = 500;
    uint32_t rcLoopMilli					= 10;
};

//=========================================================================
// IMU config
//=========================================================================

// NB: angles are in tenths of a degree

struct ImuConfig {

    uint16_t accel1G			    = 4096;
    float    accelLpfFactor	        = 4.f;
    int32_t  accelZDeadband	        = 40;
    int32_t  accelXyDeadband        = 40;
    float    accelzLpfCutoff        = 5.f;
    uint32_t calibratingGyroMilli	= 3500;
    uint32_t calibratingAccelMilli	= 1400;
    float    gyroCmpfFactor		    = 600.f;
    float    gyroScale			    = 16.4f;  // for Invensense MPU 
    uint32_t loopMicro				= 3500;
    uint16_t maxAngleInclination    = 500; 
    uint16_t maxArmingAngle			= 250;		 
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

    uint16_t mincheck	= 1100;
    uint16_t maxcheck	= 1900;
    int16_t expo8		= 65;
    int16_t rate8		= 90;
    int8_t  thrMid8		= 50;
    int32_t thrExpo8	 = 0;
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
    LoopConfig loop;
    ImuConfig imu;
    RcConfig rc;
    PidConfig pid;
    PwmConfig pwm;
    InitConfig init;
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
