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

            // Loop timing
            Timer openTimer   = Timer(100);
            Timer innerTimer  = Timer(285);

            // Passed to Hackflight::init() for a particular board and receiver
            Board      * board;
            Receiver   * receiver;
            Stabilizer * stabilizer;

            // Eventually we might want to support mixers for different kinds of configurations (tricopter, etc.)
            Mixer      mixer;

            // Vehicle state
            float eulerAngles[3];
            float gyroRates[3];
            bool armed;

            // Auxiliary switch state for change detection
            uint8_t  auxState;

            // Safety
            bool     failsafe;

            // Support for headless mode
            float    yawInitial;

            void openLoop(void)
            {
                // Update Receiver demands, passing yaw angle for headless mode
                receiver->update(eulerAngles[AXIS_YAW] - yawInitial);

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

                    board->handleAuxSwitch(receiver->demands);
                }

                // Set LED based on arming status
                board->showArmedStatus(armed);

                // Do serial comms
                board->doSerialComms(eulerAngles, armed, receiver, &mixer);

            } // openLoop

            void innerLoop(void)
            {
                // Start with demands from receiver
                demands_t demands;
                memcpy(&demands, &receiver->demands, sizeof(demands_t));

                // Get Euler angles, gyro rates from board
                board->getImu(eulerAngles, gyroRates);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (eulerAngles[AXIS_YAW] < 0) {
                    eulerAngles[AXIS_YAW] += 2*M_PI;
                }

                // Run stabilization to get updated demands
                stabilizer->updateDemands(eulerAngles, gyroRates, demands);

                // Modify demands based on extra PID controllers
                board->runPidControllers(armed, demands);

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

                // Failsafe
                if (armed && receiver->lostSignal()) {
                    mixer.cutMotors();
                    armed = false;
                    failsafe = true;
                    board->showArmedStatus(false);
                }

            } // innerLoop

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
                // Grab current time for various loops
                uint32_t currentTime = (uint32_t)board->getMicroseconds();

                // Open (slow, "outer") loop: respond to receiver demands
                if (openTimer.checkAndUpdate(currentTime)) {
                    openLoop();
                }

                // Closed (fast, "inner") loop: respond to PID control
                if (innerTimer.checkAndUpdate(currentTime)) {
                    innerLoop();
                }

            } // update

     }; // class Hackflight

} // namespace
