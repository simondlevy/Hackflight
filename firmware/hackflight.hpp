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


class TimedTask {

    private:

        uint32_t usec;
        uint32_t period;

    public:

        void init(uint32_t _period) {

            this->period = _period;
            this->usec = 0;
        }

        bool checkAndUpdate(uint32_t currentTime) {

            bool result = (int32_t)(currentTime - this->usec) >= 0;

            if (result)
                this->update(currentTime);

            return result;
        }

        void update(uint32_t currentTime) {

            this->usec = currentTime + this->period;
        }

        bool check(uint32_t currentTime) {

            return (int32_t)(currentTime - this->usec) >= 0;
        }
};


class Hackflight {

    private:

        class IMU        imu;
        class RC         rc;
        class Mixer      mixer;
        class MSP        msp;
        class Stabilize  stab;
        class Board      board;

        class TimedTask imuTask;
        class TimedTask rcTask;
        class TimedTask accelCalibrationTask;

        uint32_t imuLooptimeUsec;
        uint16_t calibratingGyroCycles;
        uint16_t calibratingAccCycles;
        uint16_t calibratingG;
        bool     haveSmallAngle;
        bool     armed;

    public:

        void initialize(void);

        void update(void);
};

inline void Hackflight::initialize(void)
{
    uint16_t acc1G;
    float    gyroScale;
    uint32_t looptimeUsec;
    uint32_t gyroCalibrationMsec;

    // Get particulars for board
    Board::init(acc1G, gyroScale, looptimeUsec, gyroCalibrationMsec);

    this->imuLooptimeUsec = looptimeUsec;

    // compute cycles for calibration based on board's time constant
    this->calibratingGyroCycles = (uint16_t)(1000. * gyroCalibrationMsec / this->imuLooptimeUsec);
    this->calibratingAccCycles  = (uint16_t)(1000. * CONFIG_CALIBRATING_ACC_MSEC  / this->imuLooptimeUsec);

    // initialize our external objects with objects they need
    this->stab.init(&this->rc, &this->imu);
    this->imu.init(acc1G, gyroScale, this->calibratingGyroCycles, this->calibratingAccCycles);
    this->mixer.init(&this->rc, &this->stab); 

    // ensure not armed
    this->armed = false;

    // sleep for 100ms
    Board::delayMilliseconds(100);

    // flash the LEDs to indicate startup
    Board::ledRedOff();
    Board::ledGreenOff();
    for (uint8_t i = 0; i < 10; i++) {
        Board::ledRedOn();
        Board::ledGreenOn();
        Board::delayMilliseconds(50);
        Board::ledRedOff();
        Board::ledGreenOff();
        Board::delayMilliseconds(50);
    }

    // intialize the R/C object
    this->rc.init();

    // always do gyro calibration at startup
    this->calibratingG = this->calibratingGyroCycles;

    // assume shallow angle (no accelerometer calibration needed)
    this->haveSmallAngle = true;

    // initializing timing tasks
    this->imuTask.init(this->imuLooptimeUsec);
    this->rcTask.init(CONFIG_RC_LOOPTIME_MSEC * 1000);
    this->accelCalibrationTask.init(CONFIG_CALIBRATE_ACCTIME_MSEC * 1000);

    // initialize MSP comms
    this->msp.init(&this->imu, &this->mixer, &this->rc, &this->board);

    // do any extra initializations (baro, sonar, etc.)
    this->board.extrasInit(&msp);

} // intialize


inline void Hackflight::update(void)
{
    static bool     accCalibrated;
    static uint16_t calibratingA;
    static uint32_t currentTime;

    bool rcSerialReady = Board::rcSerialReady();

    if (this->rcTask.checkAndUpdate(currentTime) || rcSerialReady) {

        // update RC channels
        this->rc.update(&this->board);

        rcSerialReady = false;

        // useful for simulator
        if (this->armed)
            Board::showAuxStatus(this->rc.auxState());

        // when landed, reset integral component of PID
        if (this->rc.throttleIsDown()) 
            this->stab.resetIntegral();

        if (this->rc.changed()) {

            if (this->armed) {      // actions during armed

                // Disarm on throttle down + yaw
                if (this->rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (this->armed) {
                        armed = false;
                        Board::showArmedStatus(this->armed);
                    }
                }
            } else {         // actions during not armed

                // gyro calibration
                if (this->rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) 
                    this->calibratingG = this->calibratingGyroCycles;

                // Arm via throttle-low / yaw-right
                if (this->rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
                    if (this->calibratingG == 0 && accCalibrated) 
                        if (!this->rc.auxState()) // aux switch must be in zero position
                            if (!this->armed) {
                                this->armed = true;
                                Board::showArmedStatus(this->armed);
                            }

                // accel calibration
                if (this->rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = this->calibratingAccCycles;

            } // not armed

        } // this->rc.changed()

        // Detect aux switch changes for hover, altitude-hold, etc.
        this->board.extrasCheckSwitch();

    } else {                    // not in rc loop

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        this->board.extrasPerformTask(taskOrder);

        taskOrder++;

        if (taskOrder >= Board::extrasGetTaskCount()) // using >= supports zero or more tasks
            taskOrder = 0;
    }

    currentTime = Board::getMicros();

    if (this->imuTask.checkAndUpdate(currentTime)) {

        this->imu.update(currentTime, this->armed, calibratingA, this->calibratingG);

        if (calibratingA > 0)
            calibratingA--;

        if (calibratingG > 0)
            calibratingG--;

        this->haveSmallAngle = 
            abs(this->imu.angle[0]) < CONFIG_SMALL_ANGLE && abs(this->imu.angle[1]) < CONFIG_SMALL_ANGLE;

        // measure loop rate just afer reading the sensors
        currentTime = Board::getMicros();

        // compute exponential RC commands
        this->rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || this->calibratingG > 0) {
            Board::ledGreenOn();
        }
        else {
            if (accCalibrated)
                Board::ledGreenOff();
            if (this->armed)
                Board::ledRedOn();
            else
                Board::ledRedOff();
        }

        // periodically update accelerometer calibration status
        static bool on;
        if (this->accelCalibrationTask.check(currentTime)) {
            if (!this->haveSmallAngle) {
                accCalibrated = false; 
                if (on) {
                    Board::ledGreenOff();
                    on = false;
                }
                else {
                    Board::ledGreenOn();
                    on = true;
                }
                this->accelCalibrationTask.update(currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // update stability PID controller 
        this->stab.update();

        // update mixer
        this->mixer.update(this->armed, &this->board);

        // handle serial communications
        this->msp.update(this->armed);

    } // IMU update

} // loop()
