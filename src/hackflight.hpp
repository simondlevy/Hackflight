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
#include "state.hpp"
#include "serialtask.hpp"

namespace hf {

    class Hackflight : public rft::RFT {

        private:

            // Serial comms task
            SerialTask _serialTask;

            // Vehicle state
            State _state;
       
         protected:

            virtual rft::State * getState(void) override
            {
                return &_state;
            }

         public:

            Hackflight(rft::Board * board, Receiver * receiver, Mixer * mixer) 
                : rft::RFT(board, receiver, mixer)
            {
            }

            void begin(bool armed=false)
            {
                RFT::begin(armed);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _olc, _actuator);

            }

            void update(void)
            {
                RFT::update();

                // Update serial comms task
                _serialTask.update();
            }

    }; // class Hackflight

} // namespace
