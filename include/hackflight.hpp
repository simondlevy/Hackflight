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

#include <cstdlib>
#include <cstdarg>
#include <cstdio>

#include "config.hpp"
#include "board.hpp"
#include "mixer.hpp"
#include "msp.hpp"
#include "common.hpp"
#include "rc.hpp"
#include "imu.hpp"
#include "stabilize.hpp"
#include "timedtask.hpp"

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


namespace hf {

class Hackflight {
    public:
        void init(Board * _board);
        void update(void);

    private:
        void blinkLedForTilt(void);
        bool gotRcUpdate(void);
        void flashLeds(const InitConfig& config);
        void updateImu(void);
        void updateReadyState(int16_t eulerAngles[3]);

    private:
        bool       armed;

        RC         rc;
        IMU        imu;
        Mixer      mixer;
        MSP        msp;
        Stabilize  stab;
        Board    * board;

        TimedTask imuTask;
        TimedTask rcTask;
        TimedTask angleCheckTask;

        bool     safeToArm;
        uint16_t maxArmingAngle;
};

/********************************************* CPP ********************************************************/

void Hackflight::init(Board * _board)
{  
    board = _board;

    // Do hardware initialization for board
    board->init();

    // Get board configuration
    const Config& config = board->getConfig();

    // Flash the LEDs to indicate startup
    flashLeds(config.init);

    // Get particulars for board
    LoopConfig loopConfig = config.loop;
    ImuConfig imuConfig = config.imu;

    // Store some for later
    maxArmingAngle = imuConfig.maxArmingAngle;

    // Initialize the IMU
    imu.init(imuConfig, board);

    // Sleep  a bit to allow IMU to catch up
    board->delayMilliseconds(config.init.delayMilli);

    // Initialize timing tasks
    imuTask.init(imuConfig.loopMicro);
    rcTask.init(loopConfig.rcLoopMilli * 1000);
    angleCheckTask.init(loopConfig.angleCheckMilli * 1000);

    // Initialize the RC receiver
    rc.init(config.rc, config.pwm, board);

    // Initialize our stabilization, mixing, and MSP (serial comms)
    stab.init(config.pid, config.imu, board);
    mixer.init(config.pwm, &rc, &stab); 
    msp.init(&imu, &mixer, &rc, board);

    // Initialize any extra stuff you want to do with your board
    board->extrasInit(&msp);

    // Ready to rock!
    armed = false;
    safeToArm = false;

} // init

void Hackflight::update(void)
{
    // If we didn't get new RC values, we can use the time to perform any extra tasks ("outer loop")
    if (!gotRcUpdate()) {

        static int taskOrder;

        board->extrasPerformTask(taskOrder);

        taskOrder++;

        if (taskOrder >= board->extrasGetTaskCount()) // using >= supports zero or more tasks
            taskOrder = 0;
    }

    // Regardless, we always update the IMU ("inner loop")
    updateImu();

} // update

bool Hackflight::gotRcUpdate(void)
{
    // if it's not time to update, and we have no new serial RC values, we're done
    if (!rcTask.checkAndUpdate(board->getMicros()) && !board->rcSerialReady()) {
        return false;
    }

    // update RC channels
    rc.update();

	debug(board, "%d %d %d %d %d", rc.data[0], rc.data[1], rc.data[2], rc.data[3], rc.data[4]);

    // useful for simulator
    if (armed) {
        board->showAuxStatus(rc.auxState());
    }

    // when landed, reset integral component of PID
    if (rc.throttleIsDown()) {
        stab.resetIntegral();
    }

    if (rc.changed()) {

        // actions during armed
        if (armed) {      

            // Disarm on throttle down + yaw
            if (rc.sticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
                if (armed) {
                    armed = false;
                    board->showArmedStatus(armed);
                }
            }

        // actions during not armed
        } else {         

            // Restart IMU calibration via throttle-low / yaw left
            if (rc.sticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
                board->imuRestartCalibration();
            }

            // Arm via throttle-low / yaw-right
            if (rc.sticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
                if (board->imuGyroCalibrated() && safeToArm) {
                    if (!rc.auxState()) // aux switch must be in zero position
                        if (!armed) {
                            armed = true;
                            board->showArmedStatus(armed);
                        }
                }
            }

        } // not armed

    } // rc.changed()

    // Detect aux switch changes for hover, altitude-hold, etc.
    board->extrasCheckSwitch();

    return true;
}

void Hackflight::updateImu(void)
{
    uint32_t currentTime = board->getMicros();

    // Special handling for EM7180 SENtral Sensor Fusion IMU
    board->imuUpdate();

    if (imuTask.checkAndUpdate(currentTime)) {

        // Compute exponential RC commands
        rc.computeExpo();

        // IMU update reads IMU raw angles and converts them to Euler angles
        imu.update(currentTime, armed);

        // Periodically update accelerometer calibration status using Euler angles
        updateReadyState(imu.eulerAngles);

        // Stabilization, mixing, and MSP are synced to IMU update.  Stabilizer also uses raw gyro values.
        stab.update(rc.command, imu.gyroRaw, imu.eulerAngles);
        mixer.update(armed, board);
        msp.update(armed);
    } 
} 

void Hackflight::updateReadyState(int16_t eulerAngles[3])
{
    // use LEDs to indicate calibration and arming status
    if (!board->imuAccelCalibrated() || !board->imuGyroCalibrated()) {
        board->ledSet(0, true);
    }
    else {
        if (safeToArm)
            board->ledSet(0, false);
        if (armed)
            board->ledSet(1, true);
        else
            board->ledSet(1, false);
    }

    // If angle too steep, restart accel calibration and flash LED
    uint32_t currentTime = board->getMicros();
    if (angleCheckTask.check(currentTime)) {
        if (!(abs(eulerAngles[0]) < maxArmingAngle && abs(eulerAngles[1]) < maxArmingAngle)) {
            safeToArm = false; 
            blinkLedForTilt();
            angleCheckTask.update(currentTime);
        } else {
            safeToArm = true;
        }
    }
}

void Hackflight::blinkLedForTilt(void)
{
    static bool on;

    if (on) {
        board->ledSet(0, false);
        on = false;
    }
    else {
        board->ledSet(0, true);
        on = true;
    }
}

void Hackflight::flashLeds(const InitConfig& config)
{
    uint32_t pauseMilli = config.ledFlashMilli / config.ledFlashCount;
    board->ledSet(0, false);
    board->ledSet(1, false);
    for (uint8_t i = 0; i < config.ledFlashCount; i++) {
        board->ledSet(0, true);
        board->ledSet(1, false);
        board->delayMilliseconds(pauseMilli);
        board->ledSet(0, false);
        board->ledSet(1, true);
        board->delayMilliseconds(pauseMilli);
    }
    board->ledSet(0, false);
    board->ledSet(1, false);
}

} // namespace
