/*
   board.cpp : Hackflight routines for running on a flight-control board

   Uses the setup(), loop() pattern from Arduino

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

#if defined(STM32)
extern "C" { 
#endif

void Hackflight::setup(void)
{
    uint16_t acc1G;
    float    gyroScale;
    uint32_t looptimeUsec;
    uint32_t calibratingGyroMsec;

    // Get particulars for board
    Board::init(acc1G, gyroScale, looptimeUsec, calibratingGyroMsec);

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

    // initialize the Hackflight object
    this->initialize(acc1G, gyroScale, looptimeUsec, calibratingGyroMsec);

} // setup

void Hackflight::loop(void)
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

        int16_t accelADC[3], gyroADC[3];

        Board::imuRead(accelADC, gyroADC);

        this->imu.update(accelADC, gyroADC, currentTime, this->armed, calibratingA, this->calibratingG);

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

        // update PIDs and compute motor values
        this->update();

        // handle serial communications
        this->msp.update(this->armed);

        // spin motors
        for (uint8_t i = 0; i < 4; i++)
            Board::writeMotor(i, this->mixer.motors[i]);

    } // IMU update

} // loop()

static Hackflight hackflight;

void setup(void)
{
    hackflight.setup();
}

void loop(void)
{
    hackflight.loop();
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


