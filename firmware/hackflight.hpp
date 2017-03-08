/*
   hackflight.hpp : general header

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>

#include "crossplatform.h"

#ifndef M_PI
#endif

#include "board.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "stabilize.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "timedtask.hpp"

#ifndef abs
#define abs(x)    ((x) > 0 ? (x) : -(x))
#define sgn(x)    ((x) > 0 ? +1 : -1)
#define constrain(val, lo, hi) (val) < (lo) ? lo : ((val) > hi ? hi : val) 
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

// Config =====================================================


#define CONFIG_CALIBRATING_ACC_MSEC                 1400

#define CONFIG_YAW_CONTROL_DIRECTION                1    // 1 or -1 
#define CONFIG_RC_LOOPTIME_MSEC                     21
#define CONFIG_CALIBRATE_ACCTIME_MSEC               500
#define CONFIG_SMALL_ANGLE                          250  // tenths of a degree
#define CONFIG_ALTITUDE_UPDATE_MSEC                 25   // based on accelerometer low-pass filter

namespace hf {

class Hackflight {

    private:

        IMU        imu;
        RC         rc;
        Mixer      mixer;
        MSP        msp;
        Stabilize  stab;
        Board    * board;

        TimedTask imuTask;
        TimedTask rcTask;
        TimedTask accelCalibrationTask;

        uint32_t imuLooptimeUsec;
        uint16_t calibratingGyroCycles;
        uint16_t calibratingAccCycles;
        uint16_t calibratingG;
        bool     haveSmallAngle;
        bool     armed;

    public:

        void init(Board * _board);

        void update(void);
};

inline void Hackflight::init(Board * _board)
{
    this->board = _board;

    uint16_t acc1G;
    float    gyroScale;
    uint32_t looptimeUsec;
    uint32_t gyroCalibrationMsec;

    // Get particulars for board
    Board::init(acc1G, gyroScale, looptimeUsec, gyroCalibrationMsec);

    imuLooptimeUsec = looptimeUsec;

    // compute cycles for calibration based on board's time constant
    calibratingGyroCycles = (uint16_t)(1000. * gyroCalibrationMsec / imuLooptimeUsec);
    calibratingAccCycles  = (uint16_t)(1000. * CONFIG_CALIBRATING_ACC_MSEC  / imuLooptimeUsec);

    // initialize our external objects with objects they need
    stab.init(&rc, &imu);
    imu.init(acc1G, gyroScale, calibratingGyroCycles, calibratingAccCycles);
    mixer.init(&rc, &stab); 

    // ensure not armed
    armed = false;

    // sleep for 100ms
    board->delayMilliseconds(100);

    // flash the LEDs to indicate startup
    board->ledRedOff();
    board->ledGreenOff();
    for (uint8_t i = 0; i < 10; i++) {
        board->ledRedOn();
        board->ledGreenOn();
        board->delayMilliseconds(50);
        board->ledRedOff();
        board->ledGreenOff();
        board->delayMilliseconds(50);
    }

    // intialize the R/C object
    rc.init();

    // always do gyro calibration at startup
    calibratingG = calibratingGyroCycles;

    // assume shallow angle (no accelerometer calibration needed)
    haveSmallAngle = true;

    // initializing timing tasks
    imuTask.init(imuLooptimeUsec);
    rcTask.init(CONFIG_RC_LOOPTIME_MSEC * 1000);
    accelCalibrationTask.init(CONFIG_CALIBRATE_ACCTIME_MSEC * 1000);

    // initialize MSP comms
    msp.init(&imu, &mixer, &rc, board);

    // do any extra initializations (baro, sonar, etc.)
    board->extrasInit(&msp);

} // intialize


inline void Hackflight::update(void)
{
    static bool     accCalibrated;
    static uint16_t calibratingA;
    static uint32_t currentTime;

    bool rcSerialReady = Board::rcSerialReady();

    if (rcTask.checkAndUpdate(currentTime) || rcSerialReady) {

        // update RC channels
        rc.update(board);

        rcSerialReady = false;

        // useful for simulator
        if (armed)
            Board::showAuxStatus(rc.auxState());

        // when landed, reset integral component of PID
        if (rc.throttleIsDown()) 
            stab.resetIntegral();

        if (rc.changed()) {

            if (armed) {      // actions during armed

                // Disarm on throttle down + yaw
                if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        Board::showArmedStatus(armed);
                    }
                }
            } else {         // actions during not armed

                // gyro calibration
                if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) 
                    calibratingG = calibratingGyroCycles;

                // Arm via throttle-low / yaw-right
                if (rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
                    if (calibratingG == 0 && accCalibrated) 
                        if (!rc.auxState()) // aux switch must be in zero position
                            if (!armed) {
                                armed = true;
                                Board::showArmedStatus(armed);
                            }

                // accel calibration
                if (rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = calibratingAccCycles;

            } // not armed

        } // rc.changed()

        // Detect aux switch changes for hover, altitude-hold, etc.
        board->extrasCheckSwitch();

    } else {                    // not in rc loop

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        board->extrasPerformTask(taskOrder);

        taskOrder++;

        if (taskOrder >= Board::extrasGetTaskCount()) // using >= supports zero or more tasks
            taskOrder = 0;
    }

    currentTime = board->getMicros();

    if (imuTask.checkAndUpdate(currentTime)) {

        imu.update(currentTime, armed, calibratingA, calibratingG);

        if (calibratingA > 0)
            calibratingA--;

        if (calibratingG > 0)
            calibratingG--;

        haveSmallAngle = 
            abs(imu.angle[0]) < CONFIG_SMALL_ANGLE && abs(imu.angle[1]) < CONFIG_SMALL_ANGLE;

        // measure loop rate just afer reading the sensors
        currentTime = board->getMicros();

        // compute exponential RC commands
        rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) {
            board->ledGreenOn();
        }
        else {
            if (accCalibrated)
                board->ledGreenOff();
            if (armed)
                board->ledRedOn();
            else
                board->ledRedOff();
        }

        // periodically update accelerometer calibration status
        static bool on;
        if (accelCalibrationTask.check(currentTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; 
                if (on) {
                    board->ledGreenOff();
                    on = false;
                }
                else {
                    board->ledGreenOn();
                    on = true;
                }
                accelCalibrationTask.update(currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // update stability PID controller 
        stab.update();

        // update mixer
        mixer.update(armed, board);

        // handle serial communications
        msp.update(armed);

    } // IMU update

} // update()

} // namespace
