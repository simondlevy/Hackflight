/*
   hackflight.cpp : Hackflight class implementation and hooks to setup(), loop()

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.c

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

#include <math.h>

#include "hackflight.hpp"

#include <string.h>

// support for timed tasks


#if defined(STM32)
extern "C" { 
#endif

void Hackflight::initialize(void)
{
    uint32_t calibratingGyroMsec;

    // Get particulars for board
    Board::init(this->imuLooptimeUsec, calibratingGyroMsec);

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

    // compute cycles for calibration based on board's time constant
    this->calibratingGyroCycles = (uint16_t)(1000. * calibratingGyroMsec / this->imuLooptimeUsec);
    this->calibratingAccCycles  = (uint16_t)(1000. * CONFIG_CALIBRATING_ACC_MSEC  / this->imuLooptimeUsec);

    // initializing timing tasks
    this->imuTask.init(this->imuLooptimeUsec);
    this->rcTask.init(CONFIG_RC_LOOPTIME_MSEC * 1000);
    this->accelCalibrationTask.init(CONFIG_CALIBRATE_ACCTIME_MSEC * 1000);
    this->altitudeEstimationTask.init(CONFIG_ALTITUDE_UPDATE_MSEC * 1000);

    // initialize our external objects with objects they need
    this->rc.init();
    stab.init(&this->rc, &this->imu);
    this->imu.init(this->calibratingGyroCycles, this->calibratingAccCycles);
    this->mixer.init(&this->rc, &this->stab); 
    this->msp.init(&this->imu, &this->mixer, &this->rc);

    // do any extra initializations (baro, sonar, etc.)
    this->board.extrasInit(&msp);

    // always do gyro calibration at startup
    this->calibratingG = this->calibratingGyroCycles;

    // assume shallow angle (no accelerometer calibration needed)
    this->haveSmallAngle = true;

    // ensure not armed
    this->armed = false;
    
} // intialize

void Hackflight::setAccelReading(int16_t linear_accel[3]) 
{
    this->imu.accelADC[0] = linear_accel[0];
    this->imu.accelADC[1] = linear_accel[1];
    this->imu.accelADC[2] = linear_accel[2];
}

void Hackflight::setGyrolReading(int16_t angular_velocity[3])
{
    this->imu.gyroADC[0] = angular_velocity[0];
    this->imu.gyroADC[1] = angular_velocity[1];
    this->imu.gyroADC[2] = angular_velocity[2];
}

void Hackflight::update(void)
{
    static bool     accCalibrated;
    static uint16_t calibratingA;
    static uint32_t currentTime;
    static uint32_t disarmTime;

    bool rcSerialReady = Board::rcSerialReady();

    if (this->rcTask.checkAndUpdate(currentTime) || rcSerialReady) {

        // update RC channels
        this->rc.update();

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
                        // Reset disarm time so that it works next time we arm the Board::
                        if (disarmTime != 0)
                            disarmTime = 0;
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

        // handle serial communications
        this->msp.update(this->armed);

        // update stability PID controller 
        this->stab.update();

        // update mixer
        this->mixer.update(this->armed);

    } // IMU update


} // loop()

static Hackflight hackflight;

void setup(void)
{
    hackflight.initialize();
}

void loop(void)
{
    hackflight.update();
}

#if defined(STM32)
} // extern "C"
#endif

void debug(const char * fmt, ...)
{
    va_list ap;       

    va_start(ap, fmt);     

    char buf[1000];

    vsprintf(buf, fmt, ap);

    Board::dump(buf);

    va_end(ap);  
}


