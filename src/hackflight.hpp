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
#include "timer.hpp"
#include "model.hpp"
#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Loop timing
            Timer innerTimer      = Timer(285);
            Timer outerTimer      = Timer(100);
            Timer angleCheckTimer = Timer(2);
            Timer altitudeTimer   = Timer(40);

            // Essential components
            Mixer      mixer;
            Stabilize  stab;

            // Multiwii Serial Protocol communications
            MSP        msp;

            // Passed to Hackflight::init() for a particular board and receiver
            Board    * board;
            Receiver * receiver;

            // XXX this should eventually be passed in as an option
            Altitude   alti;

            // Vehicle state
            vehicle_state_t state;

            // Auxiliary switch state
            uint8_t  auxState;

            // Safety
            bool     failsafe;
            bool     safeToArm;

            // Support for headless mode
            float    yawInitial;

            void outerLoop(void)
            {
                // Update Receiver demands, passing yaw angle for headless mode
                receiver->update(state.pose.orientation[AXIS_YAW].value - yawInitial);

                // When landed, reset integral component of PID
                if (receiver->throttleIsDown()) {
                    stab.resetIntegral();
                }

                // Certain actions (arming, disarming) need checking every time
                if (receiver->changed()) {

                    // actions during armed
                    if (state.armed) {      

                        // Disarm
                        if (receiver->disarming()) {
                            if (state.armed) {
                                state.armed = false;
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
                                    if (!state.armed) {
                                        yawInitial = state.pose.orientation[AXIS_YAW].value;
                                        state.armed = true;
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
                board->ledSet(state.armed);

                // Update serial comms
                msp.update(state, state.armed);

            } // outerLoop

            void innerLoop(void)
            {
                // Start with demands from receiver
                demands_t demands;
                memcpy(&demands, &receiver->demands, sizeof(demands_t));

                // Get vehicle state (minimally, Euler angles and gyro angular velocities) from board
                board->getState(&state);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state.pose.orientation[AXIS_YAW].value < 0) {
                    state.pose.orientation[AXIS_YAW].value += 2*M_PI;
                }

                // Udate altitude estimator with accelerometer data
                // XXX Should be done in hardware!
                alti.fuseWithImu(state.pose.orientation, state.armed);

                // Run stabilization to get updated demands
                stab.updateDemands(state, demands);

                // Modify demands based on extras (currently just altitude-hold)
                alti.updateDemands(demands);

                // Support motor testing from GCS
                if (!state.armed) {
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

            bool safeAngle(uint8_t axis)
            {
                return fabs(state.pose.orientation[axis].value) < stab.maxArmingAngle;
            }

            void flashLed(void)
            {
                const uint32_t ledFlashMilli = 1000;
                const uint32_t ledFlashCount = 20;

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

                // Initialize the receiver
                receiver->init();

                // Initialize our stabilization, mixing, and MSP (serial comms)
                stab.init(_model);
                mixer.init(board); 
                msp.init(&mixer, receiver, board);

                // Initialize altitude estimator, which will be used if there's a barometer
                alti.init(board, _model);

                // Start unstate.armed
                state.armed = false;
                safeToArm = false;
                failsafe = false;

            } // init

            void update(void)
            {
                // Grab current time for various loops
                uint32_t currentTime = (uint32_t)board->getMicroseconds();

                // Outer (slow) loop: respond to receiver demands
                if (outerTimer.checkAndUpdate(currentTime)) {
                    outerLoop();
                }

                // Altithude-PID task (never called in same loop iteration as Receiver update)
                else if (altitudeTimer.checkAndUpdate(currentTime)) {
                    alti.estimate(state.armed);
                }

                // Inner (fast) loop: stabilize, spin motors
                if (innerTimer.checkAndUpdate(currentTime)) {
                    innerLoop();
                }

                // Periodically check pitch, roll angle for arming readiness
                if (angleCheckTimer.ready(currentTime)) {
                    safeToArm = safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH);
                }

                // Failsafe
                if (state.armed && receiver->lostSignal()) {
                    mixer.cutMotors();
                    state.armed = false;
                    failsafe = true;
                    board->ledSet(false);
                }

            } // update

    }; // class Hackflight

} // namespace
