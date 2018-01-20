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

        private: 

            // Loop timing
            const uint32_t imuLoopFreq     = 285;
            const uint32_t rcLoopFreq      = 100;
            const uint32_t altHoldLoopFreq = 40;
            const uint32_t angleCheckFreq  = 2;

            // Arbitrary
            const uint32_t startupMilli    = 100;
            const uint32_t ledFlashMilli   = 1000;
            const uint32_t ledFlashCount   = 20;

            Mixer      mixer;
            MSP        msp;
            Stabilize  stab;
            Altitude   alti;

            Board    * board;
            Receiver * receiver;

            TimedTask innerTask;
            TimedTask outerTask;
            TimedTask angleCheckTask;
            TimedTask altitudeTask;

            bool     armed;
            bool     failsafe;
            float    yawInitial;
            uint8_t  auxState;
            float    eulerAngles[3];
            bool     safeToArm;

            void outerLoop(void)
            {
                // Update Receiver demands, passing yaw angle for headless mode
                receiver->update(eulerAngles[AXIS_YAW] - yawInitial);

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
                    } 

                    // Actions during not armed
                    else {         

                        // Arming
                        if (receiver->arming()) {

                            if (!failsafe && safeToArm) {

                                auxState = receiver->getAuxState();

                                if (!auxState) // aux switch must be in zero position
                                    if (!armed) {
                                        yawInitial = eulerAngles[AXIS_YAW];
                                        armed = true;
                                    }
                            }
                        }

                    } // not armed

                } // receiver->changed()

                // Detect aux switch changes for altitude-hold, loiter, etc.
                if (receiver->getAuxState() != auxState) {
                    auxState = receiver->getAuxState();
                    alti.handleAuxSwitch(auxState, receiver->demands.throttle);
                }

                // Set LED based on arming status
                board->ledSet(armed);

                // Update serial comms
                msp.update(eulerAngles, armed);

            } // outerLoop

            void innerLoop(void)
            {
                // Start with demands from receiver
                demands_t demands;
                memcpy(&demands, &receiver->demands, sizeof(demands_t));

                // Get Euler angles and raw gyro from board
                float gyroRadiansPerSecond[3];
                board->getImu(eulerAngles, gyroRadiansPerSecond);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (eulerAngles[AXIS_YAW] < 0) {
                    eulerAngles[AXIS_YAW] += 2*M_PI;
                }

                // Udate altitude estimator with accelerometer data
                // XXX Should be done in hardware!
                alti.fuseWithImu(eulerAngles, armed);

                // Run stabilization to get updated demands
                stab.updateDemands(eulerAngles, gyroRadiansPerSecond, demands);

                // Modify demands based on extras (currently just altitude-hold)
                alti.updateDemands(demands);

                // Support motor testing from GCS
                if (!armed) {
                    mixer.runDisarmed();
                }

                // Run mixer (spin motors) unless failsafe triggered or currently arming via throttle-down
                else if (!failsafe && !receiver->throttleIsDown()) {
                    mixer.runArmed(demands);
                }

                // Cut motors on failsafe or throttle-down
                else {
                    mixer.cutMotors();
                }

            } // innerLoop

            void checkAngle(void)
            {
                safeToArm = safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH);
            }

            bool safeAngle(uint8_t axis)
            {
                return fabs(eulerAngles[axis]) < stab.maxArmingAngle;
            }

            void flashLed(void)
            {
                uint32_t pauseMilli = ledFlashMilli / ledFlashCount;
                board->ledSet(false);
                for (uint8_t i = 0; i < ledFlashCount; i++) {
                    board->ledSet(true);
                    board->delayMilliseconds(pauseMilli);
                    board->ledSet(false);
                    board->delayMilliseconds(pauseMilli);
                }
                board->ledSet(false);
            }

        public:

            void init(Board * _board, Receiver * _receiver, Model * _model)
            {  
                board = _board;
                receiver = _receiver;

                // Do hardware initialization for board
                board->init();

                // Flash the LEDs to indicate startup
                flashLed();

                // Sleep  a bit to allow IMU to catch up
                board->delayMilliseconds(startupMilli);

                // Initialize essential timing tasks
                innerTask.init(imuLoopFreq);
                outerTask.init(rcLoopFreq);
                angleCheckTask.init(angleCheckFreq);

                // Initialize the receiver
                receiver->init();

                // Initialize our stabilization, mixing, and MSP (serial comms)
                stab.init(_model);
                mixer.init(board); 
                msp.init(&mixer, receiver, board);

                // Initialize altitude estimator, which will be used if there's a barometer
                altitudeTask.init(altHoldLoopFreq);
                alti.init(board, _model);

                // Start unarmed
                armed = false;
                safeToArm = false;
                failsafe = false;

            } // init

            void update(void)
            {
                // Grab current time for various loops
                uint32_t currentTime = (uint32_t)board->getMicros();

                // Outer (slow) loop: respond to receiver demands
                if (outerTask.checkAndUpdate(currentTime)) {
                    outerLoop();
                }

                // Altithude-PID task (never called in same loop iteration as Receiver update)
                else if (altitudeTask.checkAndUpdate(currentTime)) {
                    alti.computePid(armed);
                }

                // Inner (fast) loop: stabilize, spin motors
                if (innerTask.checkAndUpdate(currentTime)) {
                    innerLoop();
                }

                // Periodically check pitch, roll angle for arming readiness
                if (angleCheckTask.ready(currentTime)) {
                    checkAngle();
                }

                // Failsafe
                if (armed && receiver->lostSignal()) {
                    mixer.cutMotors();
                    armed = false;
                    failsafe = true;
                    board->ledSet(false);
                }

            } // update

    }; // class Hackflight

} // namespace
