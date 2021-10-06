/*
   Core Hackflight class

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "HF_sensor.hpp"
#include "HF_board.hpp"
#include "HF_pidtask.hpp"
#include "HF_pidcontroller.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"
#include "HF_state.hpp"

namespace hf {

    class HackflightPure {

        private:

            // Sensors 
            Sensor * _sensors[256] = {};
            uint8_t _sensor_count = 0;

            // Timer task for PID controllers
            PidTask _pidTask;

            void startSensors(void) 
            {
                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->begin();
                }
            }

            void checkSensors(State * state)
            {
                // Some sensors may need to know the current time
                float time = _board->getTime();

                for (uint8_t k=0; k<_sensor_count; ++k) {
                    _sensors[k]->modifyState((State *)state, time);
                }
            }

        protected:

            // Essentials
            Board * _board = NULL;
            Receiver * _receiver = NULL;
            Mixer * _mixer = NULL;
            State _state = {};


        public:

            HackflightPure(Board * board, Receiver * receiver, Mixer * mixer)
            {
                _board = board;
                _receiver = receiver;
                _mixer = mixer;

                _sensor_count = 0;
            }

            void begin(bool armed=false)
            {  
                _state.armed = armed;

                // Start the board
                _board->begin();

                // Initialize the sensors
                startSensors();

                // Initialize the open-loop controller
                _receiver->begin();

                // Start the mixer
                _mixer->begin();

            } // begin

            void update(void)
            {
                _receiver->ready();

                // Update PID controllers task
                _pidTask.update(_board, _receiver, _mixer, &_state);

                // Check sensors
                checkSensors(&_state);
            }

            void addSensor(Sensor * sensor) 
            {
                _sensors[_sensor_count++] = sensor;
            }

            void addPidController(PidController * controller) 
            {
                _pidTask.addController(controller);
            }

    }; // class HackflightPure

} // namespace hf
