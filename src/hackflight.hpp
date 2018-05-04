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
#include "debug.hpp"
#include "datatypes.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Passed to Hackflight::init() for a particular board and receiver
            Board      * board;
            Receiver   * receiver;
            Stabilizer * stabilizer;
            Mixer      * mixer;

            // Vehicle state
            vehicle_state_t state;

            // Auxiliary switch state for change detection
            uint8_t auxState;

            // Safety
            bool failsafe;

            // Support for headless mode
            float yawInitial;

            uint32_t gcount, acount, qcount, bcount, rcount;

            bool safeAngle(uint8_t axis)
            {
                return fabs(state.eulerAngles[axis]) < stabilizer->maxArmingAngle;
            }

            void checkQuaternion(void)
            {
                float q[4];

                if (board->getQuaternion(q)) {

                    state.eulerAngles[0] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
                    state.eulerAngles[1] = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
                    state.eulerAngles[2] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 

                    qcount++;

                    // Convert heading from [-pi,+pi] to [0,2*pi]
                    if (state.eulerAngles[AXIS_YAW] < 0) {
                        state.eulerAngles[AXIS_YAW] += 2*M_PI;
                    }

                    // Update stabilizer with new Euler angles
                    stabilizer->updateEulerAngles(state.eulerAngles);

                    // Do serial comms
                    board->doSerialComms(&state, receiver, mixer);
                }
            }

            void checkGyroRates(void)
            {
                float gyroRates[3];

                if (board->getGyrometer(gyroRates)) {

                    gcount++;

                    // Start with demands from receiver
                    demands_t demands;
                    memcpy(&demands, &receiver->demands, sizeof(demands_t));

                    // Run stabilization to get updated demands
                    stabilizer->modifyDemands(gyroRates, demands);

                    // Sync failsafe to gyro loop
                    checkFailsafe();

                    // Use updated demands to run motors
                    if (state.armed && !failsafe && !receiver->throttleIsDown()) {
                        mixer->runArmed(demands);
                    }
                }
            }

            void checkBarometer(void)
            {
                float pressure;
                if (board->getBarometer(pressure)) {
                    bcount++;
                }
            }

            void checkAccelerometer(void)
            {
                float accelGs[3];
                if (board->getAccelerometer(accelGs)) {
                    acount++;
                }
            }

            void checkFailsafe(void)
            {
                if (state.armed && receiver->lostSignal()) {
                    mixer->cutMotors();
                    state.armed = false;
                    failsafe = true;
                    board->showArmedStatus(false);
                }
            } 

            void checkReceiver(void)
            {
                // Acquire receiver demands, passing yaw angle for headless mode
                if (!receiver->getDemands(state.eulerAngles[AXIS_YAW] - yawInitial)) return;

                rcount++;

                // Update stabilizer with cyclic demands
                stabilizer->updateDemands(receiver->demands);

                // When landed, reset integral component of PID
                if (receiver->throttleIsDown()) {
                    stabilizer->resetIntegral();
                }

                // Disarm
                if (state.armed && receiver->disarming()) {
                    state.armed = false;
                } 

                // Arm (after lots of safety checks!)
                if (!state.armed && receiver->arming() && !auxState && !failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    state.armed = true;
                    yawInitial = state.eulerAngles[AXIS_YAW]; // grab yaw for headless mode
                }

                // Detect aux switch changes for altitude-hold, loiter, etc.
                if (receiver->demands.aux != auxState) {
                    auxState = receiver->demands.aux;
                }

                // Cut motors on throttle-down
                if (state.armed && receiver->throttleIsDown()) {
                    mixer->cutMotors();
                }

                // Set LED based on arming status
                board->showArmedStatus(state.armed);

            } // checkReceiver

        public:

            void init(Board * _board, Receiver * _receiver, Stabilizer * _stabilizer, Mixer * _mixer)
            {  
                // Store the essentials
                board = _board;
                receiver = _receiver;
                stabilizer = _stabilizer;
                mixer = _mixer;

                // Initialize the receiver
                receiver->init();

                // Tell the mixer which board to use
                mixer->board = board; 

                // Start unarmed
                state.armed = false;
                failsafe = false;

            } // init

            void update(void)
            {
                checkGyroRates();
                checkQuaternion();
                checkReceiver();
                checkAccelerometer();
                checkBarometer();
            } 

    }; // class Hackflight

} // namespace
