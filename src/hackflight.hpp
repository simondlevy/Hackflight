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

#include "pid_controllers/pid_controller.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Loop timing
            Timer innerTimer      = Timer(285);
            Timer outerTimer      = Timer(100);

            // Passed to Hackflight::init() for a particular board and receiver
            Board      * board;
            Receiver   * receiver;
            Stabilizer * stabilizer;

            // Eventually we might want to support mixers for different kinds of configurations (tricopter, etc.)
            Mixer      mixer;

            // Vehicle state
            vehicle_state_t state;

            // Auxiliary switch state for change detection
            uint8_t  auxState;

            // Safety
            bool     failsafe;

            // Support for headless mode
            float    yawInitial;

            // Support for additional PID controllers
            PIDController * pidControllers = NULL;

            void outerLoop(void)
            {
                // Update Receiver demands, passing yaw angle for headless mode
                receiver->update(state.pose.orientation[AXIS_YAW].value - yawInitial);

                // When landed, reset integral component of PID
                if (receiver->throttleIsDown()) {
                    stabilizer->resetIntegral();
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

                            if (!failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {

                                auxState = receiver->demands.aux;

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
                if (receiver->demands.aux != auxState) {

                    auxState = receiver->demands.aux;

                    for (PIDController * p = pidControllers; p; p=p->next) {

                        p->handleAuxSwitch(state, receiver->demands);
                    }
                }

                // Set LED based on arming status
                board->showArmedStatus(state.armed);

                // Do serial comms
                board->doSerialComms(&state, receiver, &mixer);

            } // outerLoop

            void innerLoop(void)
            {
                // Start with demands from receiver
                demands_t demands;
                memcpy(&demands, &receiver->demands, sizeof(demands_t));

                // Get vehicle state (minimally, Euler angles and gyro angular velocities) from board
                board->getState(state);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state.pose.orientation[AXIS_YAW].value < 0) {
                    state.pose.orientation[AXIS_YAW].value += 2*M_PI;
                }

                // Run stabilization to get updated demands
                stabilizer->updateDemands(state, demands);

                // Modify demands based on extra PID controllers
                for (PIDController * p = pidControllers; p; p=p->next) {
                    p->updateDemands(state, demands, board->getMicroseconds());
                }

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

                // Failsafe
                if (state.armed && receiver->lostSignal()) {
                    mixer.cutMotors();
                    state.armed = false;
                    failsafe = true;
                    board->showArmedStatus(false);
                }

            } // innerLoop

            bool safeAngle(uint8_t axis)
            {
                return fabs(state.pose.orientation[axis].value) < stabilizer->maxArmingAngle;
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

                // Initialize extra PID controllers
                for (PIDController * p = pidControllers; p; p=p->next) {
                    p->init();
                }

                // Start unstate.armed
                state.armed = false;
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

                // Other state estimators (never called in same loop iteration as Receiver update)
                else {
                    board->runEstimators(state, currentTime);
                }

                // Inner (fast) loop: stabilize, spin motors
                if (innerTimer.checkAndUpdate(currentTime)) {
                    innerLoop();
                }

            } // update

            void addPidController(PIDController * pidController) 
            {
                if (pidControllers == NULL) {
                    pidControllers = pidController;
                }

                else {
                    PIDController * p = pidControllers;;
                    while (true) {
                        if (p == pidController) 
                            return; // disallow repetition
                        if (p->next == NULL)
                            break;
                        p = p->next;
                    } 
                    p->next = pidController;
                }
            }

     }; // class Hackflight

} // namespace
