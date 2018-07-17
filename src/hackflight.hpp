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
#include "msp.hpp"
#include "mixer.hpp"
#include "receiver.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "state.hpp"
#include "pidcontroller.hpp"
#include "pidcontrollers/stabilizer.hpp"
#include "pidcontrollers/loiter.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Passed to Hackflight::init() for a particular board and receiver
            Board      * _board;
            Receiver   * _receiver;
            Mixer      * _mixer;

            Stabilizer * _stabilizer;
            Loiter     * _loiter;

            // Vehicle state
            State _state;

            // MSP (serial comms)
            MSP _msp;

            // Safety
            bool _failsafe;

            // Support for headless mode
            float _yawInitial;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.eulerAngles[axis]) < _stabilizer->maxArmingAngle;
            }

            void checkQuaternion(void)
            {
                float q[4];

                if (_board->getQuaternion(q)) {

                    // Update state with new quaternion to yield Euler angles
                    _state.updateQuaternion(q);

                    // Update stabilizer with new Euler angles
                    _stabilizer->updateEulerAngles(_state.eulerAngles, _receiver->flightMode());

                    // Synch serial comms to quaternion check
                    doSerialComms();
                }
            }

            void checkGyroRates(void)
            {
                float gyroRates[3];

                if (_board->getGyrometer(gyroRates)) {

                    // Update state with gyro rates
                    _state.updateGyrometer(gyroRates, _board->getMicroseconds());

                    // Start with demands from receiver
                    demands_t demands;
                    memcpy(&demands, &_receiver->demands, sizeof(demands_t));

                    // Run stabilization PID controller to get updated demands
                    _stabilizer->modifyDemands(_state, demands);

                    // Run loiter PID controller if specified
                    if (_loiter && _receiver->flightMode() == MODE_LOITER) {
                        bool loitering = _loiter->modifyDemands(_state, demands);
                        _board->flashLed(loitering);
                    }

                    // Sync failsafe to gyro loop
                    checkFailsafe();

                    // Use updated demands to run motors
                    if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
                        _mixer->runArmed(demands);
                    }
                }
            }

            void checkBarometer(void)
            {
                float pressure;
                if (_board->getBarometer(pressure)) {
                    _state.updateBarometer(pressure);
                }
            }

            void checkAccelerometer(void)
            {
                float accelGs[3];
                if (_board->getAccelerometer(accelGs)) {
                    _state.updateAccelerometer(accelGs, _board->getMicroseconds());
                }
            }

            void checkOpticalFlow(void)
            {
                float flow[2];
                if (_board->getOpticalFlow(flow)) {
                    _state.updateOpticalFlow(flow);
                }
            }

            void checkRangefinder(void)
            {
                float distance;
                if (_board->getRangefinder(distance)) {
                    _state.updateRangefinder(distance, _board->getMicroseconds());
                }
            }

            void checkFailsafe(void)
            {
                if (_state.armed && _receiver->lostSignal()) {
                    _mixer->cutMotors();
                    _state.armed = false;
                    _failsafe = true;
                    _board->showArmedStatus(false);
                }
            } 

            void checkReceiver(void)
            {
                // Acquire receiver demands, passing yaw angle for headless mode
                if (!_receiver->getDemands(_state.eulerAngles[AXIS_YAW] - _yawInitial)) return;

                // Update stabilizer with cyclic demands
                _stabilizer->updateReceiver(_receiver->demands);

                // When landed, reset integral component of PID
                if (_receiver->throttleIsDown()) {
                    _stabilizer->resetIntegral();
                }

                // Disarm
                if (_state.armed && _receiver->disarming()) {
                    _state.armed = false;
                } 

                // Arm (after lots of safety checks!)
                if (!_state.armed && _receiver->arming() && !_failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.eulerAngles[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cutMotors();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

            void doSerialComms(void)
            {
                while (_board->serialAvailableBytes() > 0) {
                    _msp.update(_board->serialReadByte());
                }

                while (_msp.availableBytes() > 0) {
                    _board->serialWriteByte(_msp.readByte());
                }

                // Support motor testing from GCS
                if (!_state.armed) {
                    _mixer->runDisarmed();
                }
            }

        public:

            void init(Board * board, Receiver * receiver, Mixer * mixer, Stabilizer * stabilizer)
            {
                init(board, receiver, mixer, stabilizer, NULL);
            }

            void init(Board * board, Receiver * receiver, Mixer * mixer, Stabilizer * stabilizer, Loiter * loiter)
            {  
                // Store the essentials
                _board      = board;
                _receiver   = receiver;
                _mixer      = mixer;

                _stabilizer = stabilizer;
                _loiter     = loiter;

                // Initialize state
                _state.init();

                // Initialize MSP (serial comms)
                _msp.init(&_state, receiver, mixer);

                // Initialize the receiver
                _receiver->init();

                // Tell the mixer which board to use
                _mixer->board = board; 

                // No failsafe yet
                _failsafe = false;

            } // init

            void addPidController(PID_Controller & pidController) 
            {
            }

            void update(void)
            {
                checkGyroRates();
                checkQuaternion();
                checkReceiver();
                checkAccelerometer();
                checkBarometer();
                checkOpticalFlow();
                checkRangefinder();
            } 

    }; // class Hackflight

} // namespace
