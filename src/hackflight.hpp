/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_board.hpp>

// #include "mspparser.hpp"
#include "receiver.hpp"
#include "state.hpp"
#include "serialtask.hpp"

#include "actuators/mixer.hpp"

#include <RFT_sensor.hpp>
#include <RFT_filters.hpp>

#include <RoboFirmwareToolkit.hpp>

namespace hf {

    class Hackflight : public rft::RFT {

        private:

            // Passed to Hackflight constructor
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

            Hackflight(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
                : rft::RFT(&_state, board, receiver, actuator)
            {
                // Store the essentials
                _receiver = receiver;
            }

            void begin(bool armed=false)
            {  
                // Initialize state
                memset(&_state, 0, sizeof(State));

                RFT::begin(armed);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _receiver, _actuator);

            } // init

            void update(void)
            {
                RFT::update();

                // Update serial comms task
                _serialTask.update();

            }

    }; // class Hackflight

} // namespace
