/*
   mw.cpp : setup() and loop() routines

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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "mw.hpp"

#ifndef PRINTF
#define PRINTF printf
#endif

#include <math.h>

// Objects we use

Board board;
IMU   imu;
RC    rc;
Mixer mixer;
PID   pid;
MSP   msp;

// utilities 

static void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    for (uint8_t r = 0; r < repeat; r++) {
        for (uint8_t i = 0; i < num; i++) {
            board.ledGreenToggle();            // switch LED state
            board.delayMilliseconds(wait);
        }
        board.delayMilliseconds(60);
    }
}

static bool check_timed_task(uint32_t usec, uint32_t currentTime) 
{

    return (int32_t)(currentTime - usec) >= 0;
}

static void update_timed_task(uint32_t * usec, uint32_t period, uint32_t currentTime) 
{
    *usec = currentTime + period;
}

static bool check_and_update_timed_task(uint32_t * usec, uint32_t period, uint32_t currentTime) 
{
    bool result = (int32_t)(currentTime - *usec) >= 0;

    if (result)
        update_timed_task(usec, period, currentTime);

    return result;
}


// values initialized in setup()

static uint16_t calibratingG;
static bool     haveSmallAngle;
//static uint32_t previousTime;

void setup(void)
{
    board.init();

    // sleep for 100ms
    board.delayMilliseconds(100);

    // flash the LEDs to indicate startup
    board.ledRedOn();
    board.ledGreenOff();
    for (uint8_t i = 0; i < 10; i++) {
        board.ledRedToggle();
        board.ledGreenToggle();
        board.delayMilliseconds(50);
    }
    board.ledRedOff();
    board.ledGreenOff();

    // initialize our RC, IMU, mixer, and PID controller
    rc.init(&board);
    imu.init(&board);
    pid.init();
    mixer.init(&board, &rc, &pid); 
    msp.init(&board, &imu, &mixer, &rc);

    // set initial time
    //previousTime = board.getMicros();

    // always do gyro calibration at startup
    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;

    // trigger accelerometer calibration requirement
    haveSmallAngle = true;

} // setup

void loop(void)
{
    static uint32_t rcTime = 0;
    static uint32_t loopTime;
    static uint32_t calibratedAccTime;
    static bool     accCalibrated;
    static bool     armed;
    static uint16_t calibratingA;
    static uint32_t currentTime;
    static uint32_t disarmTime = 0;

    if (check_and_update_timed_task(&rcTime, CONFIG_RC_LOOPTIME_USEC, currentTime)) {

        // update RC channels
        rc.update();

        // when landed, reset integral component of PID
        if (rc.throttleIsDown()) 
            pid.resetIntegral();

        if (rc.changed()) {

            if (armed) {      // actions during armed

                // Disarm on throttle down + yaw
                if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        // Reset disarm time so that it works next time we arm the board.
                        if (disarmTime != 0)
                            disarmTime = 0;
                    }
                }
            } else {            // actions during not armed

                // GYRO calibration
                if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                    calibratingG = CONFIG_CALIBRATING_GYRO_CYCLES;
                } 

                // Arm via YAW
                if ((rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)) {
                    if (calibratingG == 0 && accCalibrated) {
                        if (!armed) {         // arm now!
                            armed = true;
                        }
                    } else if (!armed) {
                        blinkLED(2, 255, 1);
                    }
                }

                // Calibrating Acc
                else if (rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    calibratingA = CONFIG_CALIBRATING_ACC_CYCLES;

            } // not armed

        } // rc.changed()

    } else {                    // not in rc loop

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        switch (taskOrder) {
            case 0:
                taskOrder++;
                //sensorsGetBaro();
            case 1:
                taskOrder++;
                //sensorsGetSonar();
            case 2:
                taskOrder++;
            case 3:
                taskOrder++;
            case 4:
                taskOrder = 0;
                break;
        }
    }

    currentTime = board.getMicros();

    if (check_and_update_timed_task(&loopTime, CONFIG_IMU_LOOPTIME_USEC, currentTime)) {

        imu.update(armed, calibratingA, calibratingG);

        haveSmallAngle = abs(imu.angle[0]) < CONFIG_SMALL_ANGLE && abs(imu.angle[1]) < CONFIG_SMALL_ANGLE;

        // measure loop rate just afer reading the sensors
        currentTime = board.getMicros();
        //previousTime = currentTime;

        // compute exponential RC commands
        rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) { 
            board.ledGreenToggle();
        } else {
            if (accCalibrated)
                board.ledGreenOff();
            if (armed)
                board.ledRedOn();
            else
                board.ledRedOff();
        }

        if (check_timed_task(calibratedAccTime, currentTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; // accelerometer not calibrated or angle too steep
                board.ledGreenToggle();
                update_timed_task(&calibratedAccTime, CONFIG_CALIBRATE_ACCTIME_USEC, currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // handle serial communications
        msp.update(armed);

        // update PID controller 
        pid.update(&rc, &imu);

        // update mixer
        mixer.update(armed);

    } // IMU update

} // loop()

#ifdef __arm__
} // extern "C"
#endif
