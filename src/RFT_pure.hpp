/*
   Core class for RoboFirmwareToolkit

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "RFT_parser.hpp"

#include "HF_sensor.hpp"
#include "HF_board.hpp"
#include "HF_pidtask.hpp"
#include "HF_pidcontroller.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"

namespace rft {

    class RFTPure {

        private:

            // Safety
            bool _safeToArm = false;

            // Sensors 
            hf::Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

            // Timer task for PID controllers
            hf::PidTask _pidTask;

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
            }

            void checkSensors(hf::State * state)
            {
                // Some sensors may need to know the current time
                float time = _board->getTime();

                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState((hf::State *)state, time);
                }
            }

            void checkReceiver(hf::State * state)
            {
                // Sync failsafe to open-loop-controller
                if (_receiver->lostSignal() && state->armed) {
                    _mixer->cut();
                    state->armed = false;
                    state->failsafe = true;
                    _board->showArmedStatus(false);
                    return;
                }

                // Check whether controller data is available
                if (!_receiver->ready()) return;

                // Disarm
                if (state->armed && !_receiver->inArmedState()) {
                    state->armed = false;
                } 

                // Avoid arming when controller is in armed state
                if (!_safeToArm) {
                    _safeToArm = !_receiver->inArmedState();
                }

                // Arm after lots of safety checks
                if (_safeToArm
                    && !state->armed
                    && !state->failsafe 
                    && state->safeToArm()
                    && _receiver->inactive()
                    && _receiver->inArmedState()
                    ) {
                    state->armed = true;
                }

                // Cut motors on inactivity
                if (state->armed && _receiver->inactive()) {
                    _mixer->cut();
                }

                // Set LED based on arming status
                _board->showArmedStatus(state->armed);

            } // checkReceiver

        protected:

            // Essentials
            hf::Board * _board = NULL;
            hf::Receiver * _receiver = NULL;
            hf::Mixer * _mixer = NULL;

            RFTPure(hf::Board * board, hf::Receiver * receiver, hf::Mixer * mixer)
            {
                _board = board;
                _receiver = receiver;
                _mixer = mixer;

                _sensor_count = 0;
            }

            void begin(void)
            {  
                // Start the board
                _board->begin();

                // Initialize the sensors
                startSensors();

                // Initialize the open-loop controller
                _receiver->begin();

                // Start the mixer
                _mixer->begin();

            } // begin

            void update(hf::State * state)
            {
                // Grab control signal if available
                checkReceiver(state);

                // Update PID controllers task
                _pidTask.update(_board, _receiver, _mixer, state);

                // Check sensors
                checkSensors(state);
            }

        public:

            void addSensor(hf::Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(hf::PidController * controller) 
            {
                _pidTask.addController(controller);
            }

    }; // class RFTPure

} // namespace rft
