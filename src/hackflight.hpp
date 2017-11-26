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

#include "config.hpp"
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

    public:

        void init(Board * _board, Receiver *_receiver, Model * _model);
        void update(void);

    private:

        void blinkLedForTilt(void);
        void flashLeds(const InitConfig& config);
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
        float    eulerAnglesRadians[3];
        bool     safeToArm;
        float    maxArmingAngleRadians;
};

/********************************************* CPP ********************************************************/

void Hackflight::init(Board * _board, Receiver * _receiver, Model * _model)
{  
    board = _board;
    receiver = _receiver;

    Config config;

    // Do hardware initialization for board
    board->init(config);

    // Flash the LEDs to indicate startup
    flashLeds(config.init);

    // Convert max arming angle to radians for use later
    maxArmingAngleRadians = M_PI * config.imu.maxArmingAngleDegrees / 180.;

    // Sleep  a bit to allow IMU to catch up
    board->delayMilliseconds(config.init.delayMilli);

    // Initialize timing tasks
    imuTask.init(config.loop.imuLoopMicro);
    rcTask.init(config.loop.rcLoopMilli * 1000);
    angleCheckTask.init(config.loop.angleCheckMilli * 1000);
    altitudeTask.init(config.loop.altHoldLoopMilli * 1000);

    // Initialize the receiver
    receiver->init(config.receiver);

    // Initialize our stabilization, mixing, and MSP (serial comms)
    stab.init(config.stabilize, config.imu, _model);
    mixer.init(receiver, &stab, board); 
    msp.init(&mixer, receiver, board);

    // Initialize altitude estimator, which will be used if there's a barometer
    alti.init(config.altitude, board, _model);

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
    else if (board->extrasHaveBaro() && altitudeTask.checkAndUpdate(currentTime)) {
        alti.computePid(armed);
    }

    // Failsafe
    if (receiver->lostSignal()) {
        mixer.cutMotors();
        board->ledSet(0, false);
        return;
    }

    // Polling for EM7180 SENtral Sensor Fusion IMU
    board->extrasImuPoll();

    // Inner (fast) loop: update IMU
    if (imuTask.checkAndUpdate(currentTime)) {
        updateImu();
    }

} // update

void Hackflight::updateRc(void)
{
    // Update Receiver channels
    receiver->update();

    /*
    Debug::printf("%+2.2f  %+2.2f  %+2.2f  %+2.2f %d\n",
           receiver->demandThrottle, receiver->demandRoll, receiver->demandPitch, receiver->demandYaw,
          receiver->getAuxState());*/
    //Debug::printf("%d\n", armed);

    // When landed, reset integral component of PID
    if (receiver->throttleIsDown()) {
        stab.resetIntegral();
    }

    // Certain actions (arming, disarming) need checking every time
    if (receiver->changed()) {

        // actions during armed
        if (armed) {      

            // Disarm on throttle down + yaw
            if (receiver->sticks == Receiver::THR_LO + Receiver::YAW_LO + Receiver::PIT_CE + Receiver::ROL_CE) {
                if (armed) {
                    armed = false;
                }
            }

        // Actions during not armed
        } else {         

            // Arm via throttle-low / yaw-right
            if (receiver->sticks == Receiver::THR_LO + Receiver::YAW_HI + Receiver::PIT_CE + Receiver::ROL_CE) {
    
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

    // Detect aux switch changes for altitude-hold
    if (receiver->getAuxState() != auxState) {
        auxState = receiver->getAuxState();
        if (board->extrasHaveBaro()) {
            if (auxState > 0) {
                //alti.start(receiver->demandThrottle);
            }
            else {
                //alti.stop();
            }
        }
    }
}

void Hackflight::updateImu(void)
{
    // Compute exponential Receiver commands
    receiver->computeExpo();

    // Get Euler angles and raw gyro from board
    float gyroRadiansPerSecond[3];
    board->getImu(eulerAnglesRadians, gyroRadiansPerSecond);

    // Convert heading from [-pi,+pi] to [0,2*pi]
    if (eulerAnglesRadians[AXIS_YAW] < 0) {
        eulerAnglesRadians[AXIS_YAW] += 2*M_PI;
    }

    // Update status using Euler angles
    updateReadyState();

    // If barometer avaialble, update accelerometer for altitude fusion, then modify throttle demand
    if (board->extrasHaveBaro()) {
        alti.updateAccelerometer(eulerAnglesRadians, armed);
        //alti.modifyThrottleDemand(receiver->demandThrottle);
    }

    /*
    Debug::printf("%f %f %f %f %d\n", 
            receiver->demandThrottle, receiver->demandRoll, receiver->demandPitch, receiver->demandYaw,
            receiver->getAuxState());*/

    // Stabilization is synced to IMU update.  Stabilizer also uses RC demands and raw gyro values.
    stab.update(receiver->demandRoll, receiver->demandPitch, receiver->demandYaw, eulerAnglesRadians, gyroRadiansPerSecond);

    // Update mixer
    mixer.update(receiver->demandThrottle, stab.pidRoll, stab.pidPitch, stab.pidYaw, armed);

    // Update serial comms
    msp.update(eulerAnglesRadians, armed);
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
        if (std::abs(eulerAnglesRadians[AXIS_ROLL])  > maxArmingAngleRadians ||
            std::abs(eulerAnglesRadians[AXIS_PITCH]) > maxArmingAngleRadians) {
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
