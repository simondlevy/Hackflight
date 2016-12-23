/*
   hackflight.cpp : setup() and loop() routines

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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "hackflight.hpp"

#include <string.h>

// Objects we use

static IMU        imu;
static RC         rc;
static Mixer      mixer;
static MSP        msp;
//static Baro       baro;
static Sonars     sonars;
static Hover      hover;
static StabilizeMultiwii  stab;
//static StabilizeLuxFloat  stab2;

// support for timed tasks

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


// values initialized in setup()

static TimedTask imuTask;
static TimedTask rcTask;
static TimedTask accelCalibrationTask;
static TimedTask altitudeEstimationTask;

static uint32_t imuLooptimeUsec;
static uint16_t calibratingGyroCycles;
static uint16_t calibratingAccCycles;
static uint16_t calibratingG;
static bool     haveSmallAngle;
static bool     armed;

// LED support

static bool ledGreenOn;

static void toggleGreenLED(void)
{
    if (ledGreenOn) {
        Board::ledSetState(0, true);
        ledGreenOn = false;
    }
    else {
        Board::ledSetState(0, false);
        ledGreenOn = true;
    }
}

void setup(void)
{
    uint32_t calibratingGyroMsec;

    // Get particulars for board
    Board::init(imuLooptimeUsec, calibratingGyroMsec);

    // sleep for 100ms
    Board::delayMilliseconds(100);

    // Turn off LEDs to start
    Board::ledSetState(0, false);
    Board::ledSetState(1, false);
    ledGreenOn = false;

    // flash the green LED to indicate startup
    for (uint8_t i = 0; i < 10; i++) {
        Board::delayMilliseconds(50);
        toggleGreenLED();
    }
    Board::ledSetState(0, false);

    // compute cycles for calibration based on board's time constant
    calibratingGyroCycles = (uint16_t)(1000. * calibratingGyroMsec / imuLooptimeUsec);
    calibratingAccCycles  = (uint16_t)(1000. * CONFIG_CALIBRATING_ACC_MSEC  / imuLooptimeUsec);

    // initializing timing tasks
    imuTask.init(imuLooptimeUsec);
    rcTask.init(CONFIG_RC_LOOPTIME_MSEC * 1000);
    accelCalibrationTask.init(CONFIG_CALIBRATE_ACCTIME_MSEC * 1000);
    altitudeEstimationTask.init(CONFIG_ALTITUDE_UPDATE_MSEC * 1000);

    // attempt to initialize barometer, sonars
    //baro.init();
    sonars.init();

    // initialize our external objects with objects they need
    rc.init();
    stab.init(&rc, &imu);
    imu.init(calibratingGyroCycles, calibratingAccCycles);
    mixer.init(&rc, &stab); 
    msp.init(&imu, &hover, &mixer, &rc, &sonars);
    hover.init(&imu, &sonars, &rc);

    // always do gyro calibration at startup
    calibratingG = calibratingGyroCycles;

    // assume shallow angle (no accelerometer calibration needed)
    haveSmallAngle = true;

    // ensure not armed
    armed = false;
    
} // setup

void loop(void)
{
    static bool     accCalibrated;
    static uint16_t calibratingA;
    static uint32_t currentTime;
    static uint32_t disarmTime;

    bool rcSerialReady = Board::rcSerialReady();

    if (rcTask.checkAndUpdate(currentTime) || rcSerialReady) {

        // update RC channels
        rc.update();

        rcSerialReady = false;

        //debug("%4d %4d %4d %4d %4d\n", rc.data[0], rc.data[1], rc.data[2], rc.data[3], rc.data[4]);

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
                        // Reset disarm time so that it works next time we arm the Board::
                        if (disarmTime != 0)
                            disarmTime = 0;
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

        // Switch to alt-hold when switch moves to position 1 or 2
        hover.checkSwitch();

    } else {                    // not in rc loop

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        switch (taskOrder) {
            case 0:
                //if (baro.available())
                //    baro.update();
                taskOrder++;
                break;
            case 1:
                if (sonars.available() && altitudeEstimationTask.checkAndUpdate(currentTime)) {
                    hover.updateAltitudePid();
                }
                taskOrder++;
                break;
            case 2:
                if (sonars.available())
                    sonars.update();
                taskOrder++;
                break;
            case 3:
                taskOrder++;
                break;
            case 4:
                taskOrder = 0;
                break;
        }
    }

    currentTime = Board::getMicros();

    if (imuTask.checkAndUpdate(currentTime)) {


        imu.update(currentTime, armed, calibratingA, calibratingG);

        haveSmallAngle = abs(imu.angle[0]) < CONFIG_SMALL_ANGLE && abs(imu.angle[1]) < CONFIG_SMALL_ANGLE;

        // measure loop rate just afer reading the sensors
        currentTime = Board::getMicros();

        // compute exponential RC commands
        rc.computeExpo();

        // use LEDs to indicate calibration status
        if (calibratingA > 0 || calibratingG > 0) 
            Board::ledSetState(0, true);
        else {
            if (accCalibrated)
                Board::ledSetState(0, false);
            if (armed)
                Board::ledSetState(1, true);
            else
                Board::ledSetState(1, false);
        }

        // periodically update accelerometer calibration status
        if (accelCalibrationTask.check(currentTime)) {
            if (!haveSmallAngle) {
                accCalibrated = false; 
                toggleGreenLED();
                accelCalibrationTask.update(currentTime);
            } else {
                accCalibrated = true;
            }
        }

        // handle serial communications
        msp.update(armed);

        // perform hover tasks (alt-hold etc.)
        hover.perform();

        // update stability PID controller 
        stab.update();

        // update mixer
        mixer.update(armed);

    } // IMU update

} // loop()

void debug(const char * fmt, ...)
{
    va_list ap;       

    va_start(ap, fmt);     

    char buf[1000];

    vsprintf(buf, fmt, ap);

    for (char * p = buf; *p; p++)
        Board::serialDebugByte(*p);

    va_end(ap);  
}
#
#ifdef __arm__
} // extern "C"
#endif
