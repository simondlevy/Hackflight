/*
   hackflight.hpp : general header, plus init and update methods

   Copyright (c) 2018 Simon D. Levy

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
#include "pidcontroller.hpp"
#include "pidcontrollers/stabilizer.hpp"
#include "sensors/peripheral.hpp"
#include "sensors/gyrometer.hpp"
#include "sensors/quaternion.hpp"

namespace hf {

    class Hackflight {

        private: 

            // Passed to Hackflight::init() for a particular build
            Board      * _board;
            Receiver   * _receiver;
            Stabilizer * _stabilizer;
            Mixer      * _mixer;

            // PID controllers
            PID_Controller * _pid_controllers[256];
            uint8_t _pid_controller_count;

            // Mandatory sensors on the board
            Gyrometer _gyrometer;
            Quaternion _quaternion; // not really a sensor, but we treat it like one!

            // Additional sensors 
            Sensor * _sensors[256];
            uint8_t _sensor_count;

            // Vehicle state
            state_t _state;

            // Demands sent to mixer
            demands_t _demands;
            
            // MSP (serial comms)
            MSP _msp;

            // Safety
            bool _safeToArm;
            bool _failsafe;

            // Support for headless mode
            float _yawInitial;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.eulerAngles[axis]) < _stabilizer->maxArmingAngle;
            }

            void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
                float time = _board->getTime();

                // If quaternion data ready
                if (_quaternion.ready(time)) {

                    // Update state with new quaternion to yield Euler angles
                    _quaternion.modifyState(_state, time);

                    // Update stabilizer with new Euler angles
                    _stabilizer->updateEulerAngles(_state.eulerAngles, _receiver->getAux1State());

                    // Synch serial comms to quaternion check
                    doSerialComms();
                }
            }

            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
                if (_gyrometer.ready(time)) {

                    // Update state with gyro rates
                    _gyrometer.modifyState(_state, time);

                    // For PID control, start with demands from receiver
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
                uint8_t auxState = _receiver->getAux1State();

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
                if (!_receiver->getDemands(_state.eulerAngles[AXIS_YAW] - _yawInitial)) return;

                // Update stabilizer with cyclic demands
                _stabilizer->updateReceiver(_receiver->demands, _receiver->throttleIsDown());

                // Disarm
                if (_state.armed && !_receiver->getAux2State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux2 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux2State();
                }

                // Arm (after lots of safety checks!)
                if (    _safeToArm &&
                        !_state.armed && 
                        _receiver->throttleIsDown() &&
                        _receiver->getAux2State() && 
                        !_failsafe && 
                        safeAngle(AXIS_ROLL) && 
                        safeAngle(AXIS_PITCH)) {
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
                    if (_msp.update(_board->serialReadByte())) {
                        _board->reboot(); // support "make flash" from STM32F boards
                    }
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
                for (uint8_t k=0; k<_sensor_count; ++k) {

                    Sensor * sensor = _sensors[k];

                    float time = _board->getTime();

                    if (sensor->ready(time)) {

                        sensor->modifyState(_state, time);
                    }
                }

                //Debug::printf("forward: %+2.2f    rightward: %+2.2f\n", 
                //        _state.velocityForward, _state.velocityRightward);
            }

            void add_sensor(Sensor * sensor)
            {
                _sensors[_sensor_count++] = sensor;
            }

        public:

            void init(Board * board, Receiver * receiver, Mixer * mixer, Stabilizer * stabilizer, bool armed=false)
            {  
                // Store the essentials
                _board      = board;
                _receiver   = receiver;
                _mixer      = mixer;
                _stabilizer = stabilizer;

                // Support for mandatory sensors
                addSensor(&_quaternion, board);
                addSensor(&_gyrometer, board);

                // Support adding new sensors and PID controllers
                _sensor_count = 0;
                _pid_controller_count = 0;

                // First PID controller is always stabilizer, aux state = 0
                addPidController(stabilizer, 0);

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Support safety override by simulator
                _state.armed = armed;

                // Initialize MSP (serial comms)
                _msp.init(&_state, receiver, mixer);

                // Initialize the receiver
                _receiver->init();

                // Tell the mixer which board to use
                _mixer->board = board; 

                // Setup failsafe
                _failsafe = false;

            } // init

            void addSensor(PeripheralSensor * sensor) 
            {
                add_sensor(sensor);
            }

            void addSensor(SurfaceMountSensor * sensor, Board * board) 
            {
                add_sensor(sensor);

                sensor->board = board;
            }

            void addPidController(PID_Controller * pidController, uint8_t auxState) 
            {
                pidController->auxState = auxState;

                _pid_controllers[_pid_controller_count++] = pidController;
            }

            void update(void)
            {
                // Grab control signal if available
                checkReceiver();

                // Check mandatory sensors
                checkGyrometer();

                checkQuaternion();

                // Check optional sensors
                checkSensors();
            } 

    }; // class Hackflight

} // namespace
