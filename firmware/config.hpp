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

/*
struct Config {

    struct ImuConfig {

        uint32_t imuLoopMicro;
        uint32_t calibratingGyroMilli;

        //Defaults are for MPU6050
        uint16_t acc1G = 4096;
        float gyroScale = 16.4f;
        uint32_t calibratingAccelMilli = 1400;
        uint32_t accelCalibrationPeriodMilli = 500;
        uint32_t altitudeUpdatePeriodMilli = 500;   // based on accelerometer low-pass filter

        uint16_t magneticDeclinationTenthsDegree = 0;

        //uint16_t smallAngle = 250;  // tenths of a degree

    } imu;

    struct RcConfig {
        uint32_t rcLoopMilli = 20;
    } rc;

    uint32_t initDelayMs = 100;
    uint32_t ledFlashCountOnStartup = 20;
};
*/

//=========================================================================
//RC config
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

// Define number of RC channels, and min/max PWM
#define CONFIG_RC_CHANS 8
#define CONFIG_PWM_MIN  990
#define CONFIG_PWM_MAX  2010

// For logical combinations of stick positions (low, center, high)
#define ROL_LO (1 << (2 * DEMAND_ROLL))
#define ROL_CE (3 << (2 * DEMAND_ROLL))
#define ROL_HI (2 << (2 * DEMAND_ROLL))
#define PIT_LO (1 << (2 * DEMAND_PITCH))
#define PIT_CE (3 << (2 * DEMAND_PITCH))
#define PIT_HI (2 << (2 * DEMAND_PITCH))
#define YAW_LO (1 << (2 * DEMAND_YAW))
#define YAW_CE (3 << (2 * DEMAND_YAW))
#define YAW_HI (2 << (2 * DEMAND_YAW))
#define THR_LO (1 << (2 * DEMAND_THROTTLE))
#define THR_CE (3 << (2 * DEMAND_THROTTLE))
#define THR_HI (2 << (2 * DEMAND_THROTTLE))

#define CONFIG_RC_EXPO_8                            65
#define CONFIG_RC_RATE_8                            90
#define CONFIG_THR_MID_8                            50
#define CONFIG_THR_EXPO_8                           0
#define CONFIG_MINCHECK                             1100
#define CONFIG_MAXCHECK                             1900

#define PITCH_LOOKUP_LENGTH    7
#define THROTTLE_LOOKUP_LENGTH 12

//=========================================================================
//IMU config
//=========================================================================

#define CONFIG_ACC_LPF_FACTOR     4
#define CONFIG_ACCZ_DEADBAND      40
#define CONFIG_ACCXY_DEADBAND     40
#define CONFIG_ACCZ_LPF_CUTOFF    5.0F
#define CONFIG_GYRO_CMPF_FACTOR   600    
#define CONFIG_GYRO_CMPFM_FACTOR  250  
#define CONFIG_MORON_THRESHOLD     32

//=========================================================================
//PID config
//=========================================================================

// Level (accelerometer)
static const uint8_t CONFIG_LEVEL_P          = 40;
static const uint8_t CONFIG_LEVEL_I          = 2;

// Rate (gyro): P must be positive
static const uint8_t CONFIG_RATE_PITCHROLL_P = 36;
static const uint8_t CONFIG_RATE_PITCHROLL_I = 30;
static const uint8_t CONFIG_RATE_PITCHROLL_D = 23;

// Yaw: P must be positive
static const uint8_t CONFIG_YAW_P            = 85;
static const uint8_t CONFIG_YAW_I            = 45;

// For altitude hover
#define CONFIG_HOVER_ALT_P  120
#define CONFIG_HOVER_ALT_I  45
#define CONFIG_HOVER_ALT_D  1


//=========================================================================
// STM32 reboot support
//=========================================================================

#define CONFIG_REBOOT_CHARACTER 'R'

//=========================================================================
// MISC config
//=========================================================================

#define CONFIG_MAGNETIC_DECLINATION                 0
#define CONFIG_YAW_CONTROL_DIRECTION                1    // 1 or -1 
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_MAX_ANGLE_INCLINATION                500 /* 50 degrees */

#define CONFIG_CALIBRATING_ACC_MSEC                 1400
#define CONFIG_RC_LOOPTIME_MSEC                     20
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_SMALL_ANGLE                          250  // tenths of a degree


} // namespace
