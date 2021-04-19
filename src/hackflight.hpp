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
#include "serialtask.hpp"

#include <RFT_sensor.hpp>
#include <RFT_filters.hpp>

#include <RoboFirmwareToolkit.hpp>

namespace hf {

    class Hackflight : public rft::RFT {

        private:

            // Passed to Hackflight constructor
            Mixer * _mixer = NULL;
            Receiver * _receiver = NULL;

            // Serial timer task for GCS
            SerialTask _serialTask;

            // Vehicle state
            State _state;

        protected:

            virtual bool safeStateForArming(void) override
            {
                return _state.safeToArm();
            }

        public:

            Hackflight(rft::Board * board, Receiver * receiver, Mixer * mixer)
                : rft::RFT(&_state, board, receiver, mixer)
            {
                // Store the essentials
                _receiver = receiver;
                _mixer = mixer;
            }

            void begin(bool armed=false)
            {  
                // Initialize state
                memset(&_state, 0, sizeof(State));

                RFT::begin(armed);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _receiver, _mixer);

            } // init

            void update(void)
            {
                RFT::update();

                // Update serial comms task
                _serialTask.update();

            }

    }; // class Hackflight

} // namespace
