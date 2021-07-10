/*
   Hackflight algorithm for real vehicles

   Add serial tasks for GCS and telemetry

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "hfpure.hpp"
#include "serialtask.hpp"

namespace hf {

    class Hackflight : public HackflightPure {

        private:

            // Serial timer task for GCS on main port (USB)
            SerialTask _gcsTask = SerialTask(&_state);

            // Serial timer task for telemetry on secondary port (Serial1, Serial2, ...)
            SerialTask _telemetryTask = SerialTask(&_state, true);

        public:

            void begin(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
            {  
                HackflightPure::begin(board, receiver, actuator);

                // Start serial tasks
                _gcsTask.begin(board, receiver, actuator);
                _telemetryTask.begin(board, receiver, actuator);

            }

            void update(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
            {
                HackflightPure::update(board, receiver, actuator);

                // Update serial tasks
                _gcsTask.update(receiver, actuator, &_state);
                _telemetryTask.update(receiver, actuator, &_state);
            }

    }; // class Hackflight

} // namespace hf
