/*
   Hackflight core algorithm

   Copyright (c) 2018 Simon D. Levy, Alec Singer

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

#include "sensor.hpp"
#include "board.hpp"
#include "mspparser.hpp"
#include "mixer.hpp"
#include "receiver.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "timertasks/serialtask.hpp"
#include "timertasks/pidstask.hpp"
#include "sensors/surfacemount/gyrometer.hpp"
#include "sensors/surfacemount/quaternion.hpp"
#include "sensors/mspsensor.hpp"

namespace hf {

    class Hackflight {

        private: 

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

            // Passed to Hackflight::init() for a particular build
            Board      * _board = NULL;
            Receiver   * _receiver = NULL;
            Mixer      * _mixer = NULL;

            // Supports periodic ad-hoc debugging
            Debugger _debugger;

            // PID controllers
            PidController * _pid_controllers[256] = {NULL};
            uint8_t _pid_controller_count = 0;

            // Mandatory sensors on the board
            Gyrometer _gyrometer;
            Quaternion _quaternion; // not really a sensor, but we treat it like one!

            // Timer tasks
            SerialTask _serialTask;

            // Additional sensors 
            Sensor * _sensors[256] = {NULL};
            uint8_t _sensor_count = 0;

            // Vehicle state
            state_t _state;

            // Demands sent to mixer
            demands_t _demands;

            // Safety
            bool _safeToArm = false;
            bool _failsafe = false;

            // Support for headless mode
            float _yawInitial = 0;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.rotation[axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

            void checkQuaternion(void)
            {
                // Some quaternion filters may need to know the current time
                float time = _board->getTime();

                // If quaternion data ready
                if (_quaternion.ready(time)) {

                    // Adjust quaternion values based on IMU orientation
                    _board->adjustQuaternion(_quaternion._w, _quaternion._x, _quaternion._y, _quaternion._z);

                    // Update state with new quaternion to yield Euler angles
                    _quaternion.modifyState(_state, time);

                    // Adjust Euler angles to compensate for sloppy IMU mounting
                    _board->adjustRollAndPitch(_state.rotation[0], _state.rotation[1]);
                }
            }

            void checkGyrometer(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                // If gyrometer data ready
                if (_gyrometer.ready(time)) {

                    // Adjust gyrometer values based on IMU orientation
                    _board->adjustGyrometer(_gyrometer._x, _gyrometer._y, _gyrometer._z);

                    // Update state with gyro rates
                    _gyrometer.modifyState(_state, time);

                    // For PID control, start with demands from receiver, scaling roll/pitch/yaw by constant
                    _demands.throttle = _receiver->demands.throttle;
                    _demands.roll     = _receiver->demands.roll  * _receiver->_demandScale;
                    _demands.pitch    = _receiver->demands.pitch * _receiver->_demandScale;
                    _demands.yaw      = _receiver->demands.yaw   * _receiver->_demandScale;

                    // Sync PID controllers to gyro update
                    runPidControllers();
                }
            }

            void runPidControllers(void)
            {
                // Each PID controllers is associated with at least one auxiliary switch state
                uint8_t auxState = _receiver->getAux1State();

                // Some PID controllers should cause LED to flash when they're active
                bool shouldFlash = false;

                for (uint8_t k=0; k<_pid_controller_count; ++k) {

                    PidController * pidController = _pid_controllers[k];

                    if (pidController->auxState <= auxState) {

                        pidController->modifyDemands(_state, _demands); 

                        if (pidController->shouldFlashLed()) {
                            shouldFlash = true;
                        }
                    }
                }

                // Flash LED for certain PID controllers
                _board->flashLed(shouldFlash);

                // Use updated demands to run motors
                if (_state.armed && !_failsafe && !_receiver->throttleIsDown()) {
                    _mixer->runArmed(_demands);
                }
            }

            void checkReceiver(void)
            {
                // Sync failsafe to receiver
                if (_receiver->lostSignal() && _state.armed) {
                    _mixer->cutMotors();
                    _state.armed = false;
                    _failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether receiver data is available
                if (!_receiver->getDemands(_state.rotation[AXIS_YAW] - _yawInitial)) return;

                // Update PID controllers with receiver demands
                for (uint8_t k=0; k<_pid_controller_count; ++k) {
                    _pid_controllers[k]->updateReceiver(_receiver->demands, _receiver->throttleIsDown());
                }

                // Disarm
                if (_state.armed && !_receiver->getAux2State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux2 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux2State();
                }

                // Arm (after lots of safety checks!)
                if (_safeToArm && !_state.armed && _receiver->throttleIsDown() && _receiver->getAux2State() && 
                        !_failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.rotation[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cutMotors();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

            void checkOptionalSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(_state, time);
                    }
                }
            }

            void add_sensor(Sensor * sensor)
            {
                _sensors[_sensor_count++] = sensor;
            }

            void add_sensor(SurfaceMountSensor * sensor, Board * board) 
            {
                add_sensor(sensor);

                sensor->board = board;
            }

        public:

            void init(Board * board, Receiver * receiver, Mixer * mixer, bool armed=false)
            {  
                // Store the essentials
                _board    = board;
                _receiver = receiver;
                _mixer    = mixer;

                // Timer task initializations
                _serialTask.init(board, &_state, mixer, receiver);

                // Ad-hoc debugging support
                _debugger.init(board);

                // Support for mandatory sensors
                add_sensor(&_quaternion, board);
                add_sensor(&_gyrometer, board);

                // Support adding new sensors and PID controllers
                _sensor_count = 0;

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Support safety override by simulator
                _state.armed = armed;

               // Initialize the receiver
               _receiver->begin();

                // Tell the mixer which board to use
                _mixer->board = board; 

                // Setup failsafe
                _failsafe = false;

            } // init

            void addSensor(Sensor * sensor) 
            {
                add_sensor(sensor);
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
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
                checkOptionalSensors();

                // Update timer Tasks
                _serialTask.update();
            } 

    }; // class Hackflight

} // namespace
