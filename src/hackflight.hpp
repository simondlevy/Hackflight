/*
   hackflight.hpp : general header, plus init and update methods

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cmath>

#include "board.hpp"
#include "mixer.hpp"
#include "model.hpp"
#include "msp.hpp"
#include "receiver.hpp"
#include "stabilize.hpp"
#include "altitude.hpp"
#include "timedtask.hpp"
#include "model.hpp"
#include "debug.hpp"

namespace hf {

class Hackflight {

    private: // constants

        const uint32_t imuLoopMicro       = 3500;
        const uint32_t angleCheckMilli    = 500;
        const uint32_t rcLoopMilli        = 10;
        const uint32_t altHoldLoopMilli   = 25;

        const uint32_t delayMilli         = 100;
        const uint32_t ledFlashMilli      = 1000;
        const uint32_t ledFlashCount      = 20;

    public:

        void init(Board * _board, Receiver *_receiver, Model * _model);
        void update(void);

    private:

        void blinkLedForTilt(void);
        void flashLeds(void);
        void updateRc(void);
        void updateImu(void);
        void updateReadyState(void);

        Mixer      mixer;
        MSP        msp;
        Stabilize  stab;
        Altitude   alti;

        Board    * board;
        Receiver * receiver;

        TimedTask imuTask;
        TimedTask rcTask;
        TimedTask angleCheckTask;
        TimedTask altitudeTask;

        bool     armed;
        uint8_t  auxState;
        float    eulerAngles[3];
        bool     safeToArm;
};

/********************************************* CPP ********************************************************/

void Hackflight::init(Board * _board, Receiver * _receiver, Model * _model)
{  
    board = _board;
    receiver = _receiver;

    // Do hardware initialization for board
    board->init();

    // Flash the LEDs to indicate startup
    flashLeds();

    // Sleep  a bit to allow IMU to catch up
    board->delayMilliseconds(delayMilli);

    // Initialize timing tasks
    imuTask.init(imuLoopMicro);
    rcTask.init(rcLoopMilli * 1000);
    angleCheckTask.init(angleCheckMilli * 1000);
    altitudeTask.init(altHoldLoopMilli * 1000);

    // Initialize the receiver
    receiver->init();

    // Initialize our stabilization, mixing, and MSP (serial comms)
    stab.init(_model);
    mixer.init(receiver, &stab, board); 
    msp.init(&mixer, receiver, board);

    // Initialize altitude estimator, which will be used if there's a barometer
    alti.init(board, _model);

    // Start unarmed
    armed = false;
    safeToArm = false;

} // init

void Hackflight::update(void)
{
    // Grab current time for various loops
    uint32_t currentTime = (uint32_t)board->getMicros();

    // Outer (slow) loop: update Receiver
    if (rcTask.checkAndUpdate(currentTime)) {
        updateRc();
    }

    // Altithude-PID task (never called in same loop iteration as Receiver update)
    else if (altitudeTask.checkAndUpdate(currentTime)) {
        alti.computePid(armed);
    }

    // Inner (fast) loop: update IMU
    if (imuTask.checkAndUpdate(currentTime)) {
        updateImu();
    }

    // Failsafe
    if (receiver->lostSignal()) {
        mixer.cutMotors();
        board->ledSet(0, false);
    }

} // update

void Hackflight::updateRc(void)
{
    // Update Receiver channels
    receiver->update();

    // When landed, reset integral component of PID
    if (receiver->throttleIsDown()) {
        stab.resetIntegral();
    }

    // Certain actions (arming, disarming) need checking every time
    if (receiver->changed()) {

        // actions during armed
        if (armed) {      

            // Disarm
            if (receiver->disarming()) {
                if (armed) {
                    armed = false;
                }
            }

        // Actions during not armed
        } else {         

            // Arming
            if (receiver->arming()) {
    
                if (safeToArm) {
                    auxState = receiver->getAuxState();

                    if (!auxState) // aux switch must be in zero position
                        if (!armed) {
                            armed = true;
                        }
                }
            }

        } // not armed

    } // receiver->changed()

    // Detect aux switch changes for altitude-hold, loiter, etc.
    if (receiver->getAuxState() != auxState) {
        auxState = receiver->getAuxState();
        alti.handleAuxSwitch(auxState, receiver->demands[Receiver::DEMAND_THROTTLE]);
    }
}

void Hackflight::updateImu(void)
{
    // Compute exponential Receiver commands, passing yaw angle for headless mode
    receiver->computeExpo(eulerAngles[AXIS_YAW]);

    // Get Euler angles and raw gyro from board
    float gyroRadiansPerSecond[3];
    board->getImu(eulerAngles, gyroRadiansPerSecond);

    // Convert heading from [-pi,+pi] to [0,2*pi]
    if (eulerAngles[AXIS_YAW] < 0) {
        eulerAngles[AXIS_YAW] += 2*M_PI;
    }

    // Update status using Euler angles
    updateReadyState();

    // Udate altitude and modify throttle demand
    alti.update(eulerAngles, armed, receiver->demands[Receiver::DEMAND_THROTTLE]);

    // Stabilization is synced to IMU update.  Stabilizer also uses RC demands and raw gyro values.
    stab.update(receiver->demands, eulerAngles, gyroRadiansPerSecond);

    // Update mixer
    mixer.update(receiver->demands[Receiver::DEMAND_THROTTLE], stab.pidRoll, stab.pidPitch, stab.pidYaw, armed);

    // Update serial comms
    msp.update(eulerAngles, armed);
} 

void Hackflight::updateReadyState(void)
{
    if (safeToArm)
        board->ledSet(0, false);
    if (armed)
        board->ledSet(1, true);
    else
        board->ledSet(1, false);

    // If angle too steep, flash LED
    uint32_t currentTime = (uint32_t)board->getMicros();
    if (angleCheckTask.ready(currentTime)) {
        if (fabs(eulerAngles[AXIS_ROLL])  > stab.maxArmingAngle || fabs(eulerAngles[AXIS_PITCH]) > stab.maxArmingAngle) {
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

void Hackflight::flashLeds(void)
{
    uint32_t pauseMilli = ledFlashMilli / ledFlashCount;
    board->ledSet(0, false);
    board->ledSet(1, false);
    for (uint8_t i = 0; i < ledFlashCount; i++) {
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
