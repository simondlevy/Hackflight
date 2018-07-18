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

#include "sensor.hpp"
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

            // PID controllers
            PID_Controller * _pid_controllers[256];
            uint8_t _pid_controller_count;

            // Sensors 
            Sensor * _sensors[256];
            uint8_t _sensor_count;

            // Stabilizer is mandatory and is always the first PID controller
            Stabilizer * _stabilizer;

            // Vehicle state
            State _state;

            // Demands sent to mixer
            demands_t _demands;
            
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
                    _stabilizer->updateEulerAngles(_state.eulerAngles, _receiver->getAuxState());

                    // Synch serial comms to quaternion check
                    doSerialComms();
                }
            }

            void checkGyrometer(void)
            {
                float gyroRates[3];

                if (_board->getGyrometer(gyroRates)) {

                    // Update state with gyro rates
                    _state.updateGyrometer(gyroRates, _board->getTime());

                    // Start with demands from receiver
                    memcpy(&_demands, &_receiver->demands, sizeof(demands_t));

                    // Synch PID controllers to gyro update
                    runPidControllers();

                    // Sync failsafe to gyro loop
                    checkFailsafe();

                    // Use updated demands to run motors
                    if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
                        _mixer->runArmed(_demands);
                    }
                }
            }

            void runPidControllers(void)
            {
                // Each PID controllers is associated with at least one auxiliary switch state
                uint8_t auxState = _receiver->getAuxState();

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PID_Controller * pidController = _pid_controllers[k];

                    // XXX we should allow associating PID controllers with particular aux states
                    if (pidController->auxState <= auxState) {  

                        if (pidController->modifyDemands(_state, _demands) && pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
                _board->flashLed(shouldFlash);
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
                    _state.updateRangefinder(distance, _board->getTime());
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
                _stabilizer->updateReceiver(_receiver->demands, _receiver->throttleIsDown());

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

            void checkSensors(void)
            {
                // XXX these need to be subclasses of Sensor
                checkOpticalFlow();
                checkRangefinder();

                for (uint8_t k=0; k<_sensor_count; ++k) {

                    Sensor * sensor = _sensors[k];

                    if (sensor->ready()) {
                        sensor->modifyState(_state, _board->getTime());
                    }
                }
            }

        public:

            void init(Board * board, Receiver * receiver, Mixer * mixer, Stabilizer * stabilizer)
            {  
                // Store the essentials
                _board      = board;
                _receiver   = receiver;
                _mixer      = mixer;
                _stabilizer = stabilizer;

                // Support adding new sensors and PID controllers
                _sensor_count = 0;
                _pid_controller_count = 0;

                // First PID controller is always stabilizer, aux state = 0
                addPidController(stabilizer, 0);

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

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PID_Controller * pidController, uint8_t auxState) 
            {
                pidController->auxState = auxState;

                _pid_controllers[_pid_controller_count++] = pidController;
            }

            void update(void)
            {
                checkReceiver();

                checkGyrometer();
                checkQuaternion();

                checkSensors();
            } 

    }; // class Hackflight

} // namespace
