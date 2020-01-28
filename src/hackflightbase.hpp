/*
   Hackflight core algorithm: receiver, sensors, PID controllers

   Copyright (c) 200 Simon D. Levy

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

#include "debugger.hpp"
#include "board.hpp"
#include "demander.hpp"
#include "receiver.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "sensors/surfacemount.hpp"

namespace hf {

    class HackflightBase {

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

            // PID controllers
            PidController * _pid_controllers[256] = {NULL};
            uint8_t _pid_controller_count = 0;

            // Supports periodic ad-hoc debugging
            Debugger _debugger;

            // Mixer or receiver proxy
            Demander * _demander = NULL;

            // Sensors 
            Sensor * _sensors[256] = {NULL};
            uint8_t _sensor_count = 0;

            // Safety
            bool _safeToArm = false;
            bool _failsafe = false;

            // Support for headless mode
            float _yawInitial = 0;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.rotation[axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

        protected:

            Board      * _board    = NULL;
            Receiver   * _receiver = NULL;

            // Vehicle state
            state_t _state;

            // Demands sent to mixer
            demands_t _demands;

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

            void add_sensor(SurfaceMountSensor * sensor, IMU * imu) 
            {
                add_sensor(sensor);

                sensor->imu = imu;
            }

            void init(Board * board, Receiver * receiver, Demander * demander)
            {  
                // Store the essentials
                _board    = board;
                _receiver = receiver;
                _demander = demander;

                // Ad-hoc debugging support
                _debugger.init(board);

                // Support adding new sensors and PID controllers
                _sensor_count = 0;

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Initialize the receiver
                _receiver->begin();

                // Setup failsafe
                _failsafe = false;
            }

            void addSensor(Sensor * sensor) 
            {
                add_sensor(sensor);
            }

            void runPidControllers(void)
            {
                // Start with demands from receiver, scaling roll/pitch/yaw by constant
                _demands.throttle = _receiver->demands.throttle;
                _demands.roll     = _receiver->demands.roll  * _receiver->_demandScale;
                _demands.pitch    = _receiver->demands.pitch * _receiver->_demandScale;
                _demands.yaw      = _receiver->demands.yaw   * _receiver->_demandScale;

                // Each PID controllers is associated with at least one auxiliary switch state
                uint8_t auxState = _receiver->getAux2State();

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
                    _demander->run(_demands);
                }
            }

            void checkReceiver(void)
            {
                // Sync failsafe to receiver
                if (_receiver->lostSignal() && _state.armed) {
                    _demander->cut();
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
                if (_state.armed && !_receiver->getAux1State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux2 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux1State();
                }

                // Arm (after lots of safety checks!)
                if (_safeToArm && !_state.armed && _receiver->throttleIsDown() && _receiver->getAux1State() && 
                        !_failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.rotation[AXIS_YAW]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _demander->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

        public:

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                pidController->auxState = auxState;

                _pid_controllers[_pid_controller_count++] = pidController;
            }

     }; // class HackflightBase

} // namespace
