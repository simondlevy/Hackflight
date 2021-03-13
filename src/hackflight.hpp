/*
   Hackflight core algorithm

   Copyright (c) 2021 Simon D. Levy

   MIT License
*/

#pragma once

#include <RoboFirmwareToolkit.hpp>
#include <RFT_debugger.hpp>
#include <RFT_board.hpp>
#include <RFT_openloop.hpp>
#include <RFT_closedloop.hpp>

#include "receiver.hpp"
#include "mixer.hpp"
#include "mspparser.hpp"
#include "mavstate.hpp"
#include "serialtask.hpp"

namespace hf {

    class Hackflight : public rft::RFT {

        private:

            static constexpr float MAX_ARMING_ANGLE_DEGREES = 25.0f;

            // Safety
            bool _safeToArm = false;

            // Timer task for PID controllers
            rft::ClosedLoopTask _closedLoopTask;

            SerialTask _serialTask;

            // Vehicle state
            MavState _state;

            void checkSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    rft::Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(&_state, time);
                    }
                }
            }

            void checkOpenLoopController(void)
            {
                // Sync failsafe to open-loop-controller
                if (_olc->lostSignal() && _state.armed) {
                    _actuator->cut();
                    _state.armed = false;
                    _state.failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether open-loop controller data is available
                if (!_olc->ready()) return;

                // Disarm
                if (_state.armed && !_olc->inArmedState()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux1 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_olc->inArmedState();
                }

                // Arm (after lots of safety checks!)
                if (
                        _safeToArm &&
                        !_state.armed && 
                        _olc->inactive() && 
                        _olc->inArmedState() && 
                        !_state.failsafe && 
                        _state.safeToArm()) {
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

            Hackflight(rft::Board * board, Receiver * receiver, Mixer * mixer) 
                : rft::RFT(board, receiver, mixer)
            {
            }

            void begin(bool armed=false)
            {  
                // Start the board
                _board->begin();

                // Ad-hoc debugging support
                _debugger.begin(_board);

                // Initialize state
                memset(&_state, 0, sizeof(MavState));

                // Initialize the sensors
                startSensors();

                // Initialize the open-loop controller
                _olc->begin();

                // Initialize safety features
                _state.failsafe = false;
                _state.armed = armed;

                // Initialize timer task for PID controllers
                _closedLoopTask.begin(_board, _olc, _actuator, &_state);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _olc, _actuator);

                // Support safety override by simulator
                _state.armed = armed;

                // Start the actuator
                _actuator->begin();

            } // begin

            void addClosedLoopController(rft::ClosedLoopController * controller, uint8_t modeIndex=0) 
            {
                _closedLoopTask.addClosedLoopController(controller, modeIndex);
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
