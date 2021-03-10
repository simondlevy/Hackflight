/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

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
#include "mspparser.hpp"
#include "board.hpp"
#include "receiver.hpp"
#include "datatypes.hpp"
#include "pidcontroller.hpp"
#include "sensor.hpp"
#include "motor.hpp"
#include "mixer.hpp"
#include "actuator.hpp"
#include "timertasks/pidtask.hpp"
#include "timertasks/serialtask.hpp"

namespace hf {

    class Hackflight {

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

            // Supports periodic ad-hoc debugging
            Debugger _debugger;

            // Sensors 
            Sensor * _sensors[256] = {NULL};
            uint8_t _sensor_count = 0;

            // Safety
            bool _safeToArm = false;

            // Support for headless mode
            float _yawInitial = 0;

            // Timer task for PID controllers
            PidTask _pidTask;

            // Passed to Hackflight::begin() for a particular build
            Mixer      * _mixer    = NULL;

            // Serial timer task for GCS
            SerialTask _serialTask;

            bool safeAngle(uint8_t axis)
            {
                return fabs(_state.x[STATE_PHI+2*axis]) < Filter::deg2rad(MAX_ARMING_ANGLE_DEGREES);
            }

            Board * _board = NULL;
            Receiver * _receiver = NULL;

            // Vehicle state
            state_t _state;

            void checkSensors(void)
            {
                static uint32_t counts[2];
                /*
                static uint32_t start;
                uint32_t time = millis();
                if (time-start > 1000) {
                    Debugger::printf("q=%d  g=%d\n", counts[0], counts[1]);
                    counts[0] = 0;
                    counts[1] = 0;
                    start = time;
                }*/

                for (uint8_t k=0; k<_sensor_count; ++k) {
                    Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(_state, time);
                        if (k<2) counts[k]++;
                    }
                }
            }

            void checkReceiver(void)
            {
                // Sync failsafe to receiver
                if (_receiver->lostSignal() && _state.armed) {
                    _mixer->cut();
                    _state.armed = false;
                    _state.failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether receiver data is available
                if (!_receiver->getDemands(_state.x[STATE_PSI] - _yawInitial)) return;

                // Disarm
                if (_state.armed && !_receiver->getAux1State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux1 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux1State();
                }

                // Arm (after lots of safety checks!)
                if (_safeToArm && !_state.armed && _receiver->throttleIsDown() && _receiver->getAux1State() && 
                        !_state.failsafe && safeAngle(AXIS_ROLL) && safeAngle(AXIS_PITCH)) {
                    _state.armed = true;
                    _yawInitial = _state.x[STATE_PSI]; // grab yaw for headless mode
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
            }

         public:

            Hackflight(Board * board, Receiver * receiver, Mixer * mixer)
            {
                // Store the essentials
                _board    = board;
                _receiver = receiver;
                _mixer = mixer;

                // Support adding new sensors
                _sensor_count = 0;
            }

            void begin(bool armed=false)
            {  
                // Start the board
                _board->begin();

                // Ad-hoc debugging support
                _debugger.begin(_board);

                // Initialize state
                memset(&_state, 0, sizeof(state_t));

                // Initialize the sensors
                startSensors();

                // Initialize the receiver
                _receiver->begin();

                // Initialize safety features
                _state.failsafe = false;
                _state.armed = armed;

                // Initialize timer task for PID controllers
                _pidTask.begin(_board, _receiver, _mixer, &_state);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _receiver, _mixer);

                // Support safety override by simulator
                _state.armed = armed;

                // Start the mixer
                _mixer->begin();

            } // begin

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                _pidTask.addPidController(pidController, auxState);
            }

            void update(void)
            {
                // Grab control signal if available
                checkReceiver();

                // Update PID controllers task
                _pidTask.update();

                // Check sensors
                checkSensors();

                // Update serial comms task
                _serialTask.update();
            }

    }; // class Hackflight

} // namespace
