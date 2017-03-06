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

#include <cstdlib>
#include <cstdarg>
#include <cstdio>

#include "board.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "common.hpp"
#include "imu.hpp"
#include "rc.hpp"
#include "timedtask.hpp"

namespace hf {

class Hackflight {
public:
    void init(Board* _board);
    void update(void);

private:
    void initImuRc();
    void flashLeds(uint16_t onOffCount);
    void updateCalibrationState(bool& armed, bool& isMoving);
    void updateImu(bool armed);
    void debug(const char * fmt, ...);

private:
    bool armed;

    //objects we use
    IMU imu;
    RC rc;
    Mixer mixer;
    MSP msp;
    Stabilize stab;
    Board *board;

    // tasks that execute at specific internal
    TimedTask imuTask;
    TimedTask rcTask;
    TimedTask accelCalibrationTask;
    TimedTask altitudeEstimationTask;

    //buffers for IMU read
    int16_t gyroAdc[3], accelAdc[3];
}; //class


/********************************************* CPP ********************************************************/

void Hackflight::init(Board* _board)
{
    // =============================

    board = _board;
    initImuRc();

    board->init();

    //stab.init(&rc, &imu);
    //mixer.init(&rc, &stab); 
    //msp.init(&imu, &mixer, &rc, board);

    //TODO: can't enable this because of circuler includes
    //board->extrasInit(&msp);

    // ensure not armed
    armed = false;
}

void Hackflight::update(void)
{
    bool isMoving;
    updateCalibrationState(armed, isMoving);
    if (!isMoving)
        stab.resetIntegral();
    updateImu(armed);
    
    // handle serial communications
    msp.update(armed);

    // update stability PID controller 
    stab.update();

    // update mixer
    mixer.update(armed, board);
}

void Hackflight::initImuRc()
{
    const Config& config = board->getConfig();

    board->delayMilliseconds(config.initDelayMs);

    //flash the LEDs to indicate startup
    flashLeds(config.ledFlashCountOnStartup);

    //initialize timed tasks
    //imuTask.init(config.imu.imuLoopMicro);
    //rcTask.init(config.rc.rcLoopMilli * 1000);
    //accelCalibrationTask.init(config.imu.accelCalibrationPeriodMilli * 1000);
    //altitudeEstimationTask.init(config.imu.attitudeUpdatePeriodMilli * 1000);

    //imu.init(config.imu);
    //rc.init();
}

void Hackflight::flashLeds(uint16_t onOffCount)
{
    board->setLed(0, false);
    board->setLed(1, false);
    for (uint8_t i = 0; i < onOffCount; i++) {
        board->setLed(0, i % 2 != 0);
        board->setLed(1, i % 2 != 0);
        board->delayMilliseconds(50);
    }
}

void Hackflight::debug(const char * fmt, ...)
{
    va_list ap;       

    va_start(ap, fmt);     

    char buf[1000];

    vsprintf(buf, fmt, ap);

    board->dump(buf);

    va_end(ap);  
}

void Hackflight::updateImu(bool _armed)
{
    uint64_t currentTimeMicro = board->getMicros();

    if (imuTask.checkAndUpdate(currentTimeMicro)) {
        board->imuRead(gyroAdc, accelAdc);
        imu.update(currentTimeMicro, _armed, gyroAdc, accelAdc);


        debug("armed: %d    imu: %d %d %d\n", _armed, imu.angle[0], imu.angle[1], imu.angle[2]);

        // measure loop rate just afer reading the sensors
        currentTimeMicro = board->getMicros();

        // compute exponential RC commands
        rc.computeExpo();
    } // IMU update
}

void Hackflight::updateCalibrationState(bool& _armed, bool& isMoving)
{
    (void)_armed;

    uint64_t currentTimeMicro = board->getMicros();
    isMoving = true;

    if (rcTask.checkAndUpdate(currentTimeMicro) || board->rcSerialReady()) {

        // update RC channels
        rc.update(board);

        // useful for simulator
        if (armed)
            board->showAuxStatus(rc.auxState());

        //TODO: need to improve isMoving, currently we just return value to effect PID reset for integral component
        if (rc.throttleIsDown()) 
            isMoving = false;

        if (rc.changed()) {
            if (armed) {
                // Disarm on throttle down + yaw
                if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                    if (armed) {
                        armed = false;
                        isMoving = false;
                        board->showArmedStatus(armed);
                    }
                }
            } 
            else { //not armed
                   // gyro calibration
                if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE)
                    imu.resetCalibration(true, false);

                // Arm via throttle-low / yaw-right
                if (rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)
                    if (imu.isGyroCalibrated() && imu.isAccelCalibrated()) 
                        if (!rc.auxState()) // aux switch must be in zero position
                            if (!armed) {
                                armed = true;
                                board->showArmedStatus(armed);
                            }

                // accel calibration
                if (rc.sticks == THR_HI + YAW_LO + PIT_LO + ROL_CE)
                    imu.resetCalibration(false, true);
            } // not armed
        } // rc.changed()

          // Detect aux switch changes for hover, altitude-hold, etc.
        board->extrasCheckSwitch();
    } 
    else { //don't have RC update

        static int taskOrder;   // never call all functions in the same loop, to avoid high delay spikes

        board->extrasPerformTask(taskOrder);

        if (++taskOrder >= board->extrasGetTaskCount()) // using >= supports zero or more tasks
            taskOrder = 0;
    }

    // use LEDs to indicate calibration status
    if (!imu.isGyroCalibrated() || !imu.isAccelCalibrated())  {
        board->setLed(0, true);
    }
    else {
        if (imu.isAccelCalibrated())
            board->setLed(0, false);
        if (armed)
            board->setLed(1, true);
        else
            board->setLed(1, false);
    }
}



} //namespace
