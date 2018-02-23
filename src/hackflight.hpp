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
#include "receiver.hpp"
#include "stabilizer.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Loop timing (Hz)
            Timer openLoopTimer   = Timer(100);

            // Passed to Hackflight::init() for a particular board and receiver
            Board      * board;
            Receiver   * receiver;
            Stabilizer * stabilizer;

            // Eventually we might want to support mixers for different kinds of configurations (tricopter, etc.)
            Mixer      mixer;

            // Vehicle state
            float eulerAngles[3];
            bool armed;

            // Auxiliary switch state for change detection
            uint8_t auxState;

            // Safety
            bool failsafe;

            // Support for headless mode
            float yawInitial;

            void openLoop(void)
            {
                // Update Receiver demands, passing yaw angle for headless mode
                receiver->update(eulerAngles[AXIS_YAW] - yawInitial);

                // Update stabilizer with cyclic demands
                stabilizer->updateDemands(receiver->demands);

                // When landed, reset integral component of PID
                if (receiver->throttleIsDown()) {
                    stabilizer->resetIntegral();
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

                            if (!failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {

                                auxState = receiver->demands.aux;

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
                if (receiver->demands.aux != auxState) {

                    auxState = receiver->demands.aux;

                    //board->handleAuxSwitch(receiver->demands);
                }

                // Cut motors on failsafe or throttle-down
                if (armed) {

                    if (receiver->throttleIsDown()) {
                        mixer.cutMotors();
                    }

                    if  (receiver->lostSignal()) {
                        mixer.cutMotors();
                        armed = false;
                        failsafe = true;
                        board->showArmedStatus(false);
                    }
                }

                // Support motor testing from GCS
                else {
                    mixer.runDisarmed();
                }

                // Set LED based on arming status
                board->showArmedStatus(armed);

                // Do serial comms
                board->doSerialComms(eulerAngles, armed, receiver, &mixer);

            } // openLoop

            bool safeAngle(uint8_t axis)
            {
                return fabs(eulerAngles[axis]) < stabilizer->maxArmingAngle;
            }

        public:

            void init(Board * _board, Receiver * _receiver, Stabilizer * _stabilizer)
            {  
                // Store the essentials
                board = _board;
                receiver = _receiver;
                stabilizer = _stabilizer;

                // Do hardware initialization for board
                board->init();

                // Initialize the receiver
                receiver->init();

                // Initialize our stabilization, mixing, and MSP (serial comms)
                stabilizer->init();
                mixer.init(board); 

                // Start unarmed
                armed = false;
                failsafe = false;

            } // init

            void update(void)
            {
                if (board->getEulerAngles(eulerAngles)) {

                    // Convert heading from [-pi,+pi] to [0,2*pi]
                    if (eulerAngles[AXIS_YAW] < 0) {
                        eulerAngles[AXIS_YAW] += 2*M_PI;
                    }

                    stabilizer->updateEulerAngles(eulerAngles);
                }

                float gyroRates[3];

                if (board->getGyroRates(gyroRates)) {

                    // Start with demands from receiver
                    demands_t demands;
                    memcpy(&demands, &receiver->demands, sizeof(demands_t));

                    // Run stabilization to get updated demands
                    stabilizer->modifyDemands(gyroRates, demands);

                    // Modify demands based on extra PID controllers
                    //board->runPidControllers(demands);

                    // Use updated demands to run motors
                    if (armed && !failsafe && !receiver->throttleIsDown()) {
                        mixer.runArmed(demands);
                    }

                } //  got new gyro rates

                // Grab current time for loops
                uint32_t currentTime = board->getMicroseconds();

                // Open (slow, "outer") loop: respond to receiver demands
                if (openLoopTimer.checkAndUpdate(currentTime)) {
                    openLoop();
                }

            } // update

    }; // class Hackflight

} // namespace
