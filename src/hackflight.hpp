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

            SerialTask _serialTask;

            // Vehicle state
            MavState _state;

       
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

                // Initialize safety features
                _state.failsafe = false;
                _state.armed = armed;

                // Initialize timer task for PID controllers
                _closedLoopTask.begin(_board, _olc, _actuator, &_state);

                // Initialize serial timer task
                _serialTask.begin(_board, &_state, _olc, _actuator);

                // Support safety override by simulator
                _state.armed = armed;

            } // begin

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
