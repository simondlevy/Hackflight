/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_board.hpp>

#include "mspparser.hpp"
#include "receiver.hpp"
#include "state.hpp"
#include "mixer.hpp"
#include "timertasks/pidtask.hpp"
#include "timertasks/serialtask.hpp"

#include <rft_closedloops/pidcontroller.hpp>
#include <RFT_sensor.hpp>
#include <RFT_filters.hpp>

namespace hf {

    class Hackflight {

        private:

            // Sensors 
            rft::Sensor * _sensors[256] = {NULL};
            uint8_t _sensor_count = 0;

            // Safety
            bool _safeToArm = false;

            // Timer task for PID controllers
            PidTask _pidTask;

            // Passed to Hackflight::begin() for a particular build
            Mixer * _mixer    = NULL;

            // Serial timer task for GCS
            SerialTask _serialTask;

            void checkSensors(void)
            {
                // Some gyrometers may need to know the current time
                float time = _board->getTime();

                for (uint8_t k=0; k<_sensor_count; ++k) {
                    rft::Sensor * sensor = _sensors[k];
                    float time = _board->getTime();
                    if (sensor->ready(time)) {
                        sensor->modifyState(&_state, time);
                    }
                }
             }

            void startSensors(void)
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
             }

            rft::Board    * _board    = NULL;
            Receiver * _receiver = NULL;

            // Vehicle state
            State _state;

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
                if (!_receiver->getDemands()) {
                    return;
                }

                // Disarm
                if (_state.armed && !_receiver->getAux1State()) {
                    _state.armed = false;
                } 

                // Avoid arming if aux1 switch down on startup
                if (!_safeToArm) {
                    _safeToArm = !_receiver->getAux1State();
                }

                // Arm (after lots of safety checks!)
                if (
                        _safeToArm && 
                        !_state.armed && 
                        _receiver->throttleIsDown() && 
                        _receiver->getAux1State() && 
                        !_state.failsafe && 
                        _state.safeToArm()) {

                    _state.armed = true;
                }

                // Cut motors on throttle-down
                if (_state.armed && _receiver->throttleIsDown()) {
                    _mixer->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(_state.armed);

            } // checkReceiver

        public:

            Hackflight(rft::Board * board, Receiver * receiver, Mixer * mixer)
            {
                // Store the essentials
                _board = board;
                _receiver = receiver;
                _mixer = mixer;

                // Support adding new sensors and PID controllers
                _sensor_count = 0;
            }

            void begin(bool armed=false)
            {  
                // Initialize state
                memset(&_state, 0, sizeof(State));

                // Start the board
                _board->begin();

                // Start the sensors
                startSensors();

                // Start the receiver
                _receiver->begin();

                // Setup failsafe
                _state.failsafe = false;

                // Initialize timer task for PID controllers
                _pidTask.begin(_board, _receiver, _mixer, &_state);
 
                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _receiver, _mixer);

                // Support safety override by simulator
                _state.armed = armed;

                // Start the mixer
                _mixer->begin();

            } // init

            void addSensor(rft::Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(rft::PidController * pidController, uint8_t auxState=0) 
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
