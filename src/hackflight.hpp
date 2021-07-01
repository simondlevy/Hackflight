/*
   Hackflight core algorithm

   Copyright (c) 2020 Simon D. Levy

   MIT License
 */

#pragma once

#include <RoboFirmwareToolkit.hpp>
#include <RFT_board.hpp>
#include <RFT_openloop.hpp>
#include <RFT_actuator.hpp>
#include <rft_timertasks/closedlooptask.hpp>

#include "state.hpp"
#include "receiver.hpp"
#include "serialtask.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class Hackflight : public rft::RFT {

        private:

            // Vehicle state
            State _state;

            // Serial timer task for GCS on main port (USB)
            SerialTask _gcsTask;

            // Serial timer task for telemetry on secondary port (Serial1, Serial2, ...)
            SerialTask _telemetryTask = SerialTask(true);

        public:

            Hackflight(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
                : rft::RFT(&_state, board, receiver, actuator)
            {  
            }

            void addPidController(PidController * pidController, uint8_t auxState=0) 
            {
                RFT::addClosedLoopController(pidController, auxState);
            }

            void begin(bool armed=false)
            {  
                // Initialize state
                memset(&_state.x, 0, sizeof(_state.x));

                RFT::begin(armed);

                // Start serial tasks
                _gcsTask.begin(_board, &_state, _olc, _actuator);
                _telemetryTask.begin(_board, &_state, _olc, _actuator);

            }

            void update(void)
            {
                RFT::update();

                // Update serial tasks
                _gcsTask.update();
                _telemetryTask.update();
            }

    }; // class Hackflight

} // namespace hf
