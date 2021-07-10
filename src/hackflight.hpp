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

            Hackflight(rft::Actuator * actuator)
                : HackflightPure(actuator)
            {  
            }

            void begin(rft::Board * board, Receiver * receiver)
            {  
                HackflightPure::begin(board, receiver);

                // Start serial tasks
                _gcsTask.begin(board, receiver, _actuator);
                _telemetryTask.begin(board, receiver, _actuator);

            }

            void update(rft::Board * board, Receiver * receiver)
            {
                HackflightPure::update(board, receiver);

                // Update serial tasks
                _gcsTask.update(receiver, &_state);
                _telemetryTask.update(receiver, &_state);
            }

    }; // class Hackflight

} // namespace hf
