/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include "mspparser.hpp"
#include "receiver.hpp"
#include "state.hpp"
#include "pidcontroller.hpp"
#include "motor.hpp"
#include "serialtask.hpp"

#include <RFT_board.hpp>
#include <RFT_sensor.hpp>
#include <RFT_actuator.hpp>
#include <RFT_debugger.hpp>
#include <RFT_filters.hpp>
#include <rft_timertasks/closedlooptask.hpp>

namespace hf {

    class Hackflight {

        private:

            // Safety
            bool _safeToArm = false;

            // Sensors 
            rft::Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

            // Supports periodic ad-hoc debugging
            rft::Debugger _debugger;

            // Actuator
            rft::Actuator * _actuator = NULL;

            rft::Board  * _board = NULL;

            rft::OpenLoopController * _olc = NULL;

            // Vehicle state
            State _state;

            // Timer task for PID controllers
            rft::ClosedLoopTask _closedLoopTask;

            // Serial timer task for GCS
            SerialTask _serialTask;

            void checkSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    rft::Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState((rft::State *)&_state, time);
                    }
                }
            }

            void checkOpenLoopController(void)
            {
                // Sync failsafe to receiver
                if (_olc->lostSignal() && _state.armed) {
                    _actuator->cut();
                    _state.armed = false;
                    _state.failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether receiver data is available
                if (!_olc->ready()) return;

                // Disarm
                if (_state.armed && !_olc->inArmedState()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux1 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_olc->inArmedState();
                }

                // Arm after lots of safety checks
                if (_safeToArm
                    && !_state.armed
                    && _olc->inactive()
                    && _olc->inArmedState()
                    && !_state.failsafe
                    && _state.safeToArm()) {
                    _state.armed = true;
                }

                // Cut motors on throttle-down
                if (_state.armed && _olc->inactive()) {
                    _actuator->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkOpenLoopController

            void startSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
             }

        public:

            Hackflight(rft::Board * board, rft::OpenLoopController * olc, rft::Actuator * actuator)
            {  
                // Store the essentials
                _board = board;
                _olc = olc;
                _actuator = actuator;

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
                memset(&_state.x, 0, sizeof(_state.x));

                // Start the receiver
                _olc->begin();

                // Setup failsafe
                _state.failsafe = false;

                // Initialize timer task for PID controllers
                _closedLoopTask.begin(_board, _olc, _actuator, &_state);
 
                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _olc, _actuator);

                // Support safety override by simulator
                _state.armed = armed;

                // Start the sensors
                startSensors();

                // Tell the actuator to start the motors
                _actuator->begin();

            } // begin

            void addSensor(rft::Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                _closedLoopTask.addController(pidController, auxState);
            }

            void update(void)
            {
                // Grab control signal if available
                checkOpenLoopController();

                // Update PID controllers task
                _closedLoopTask.update();

                // Check sensors
                checkSensors();

                // Update serial comms task
                _serialTask.update();
            }

    }; // class Hackflight

} // namespace
