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

            Hackflight(Receiver * receiver, rft::Actuator * actuator)
                : HackflightPure(receiver, actuator)
            {  
            }

            void begin(rft::Board * board)
            {  
                HackflightPure::begin(board);

                // Start serial tasks
                _gcsTask.begin(board, _olc, _actuator);
                _telemetryTask.begin(board, _olc, _actuator);

            }

            void update(rft::Board * board)
            {
                HackflightPure::update(board);

                // Update serial tasks
                _gcsTask.update(&_state);
                _telemetryTask.update(&_state);
            }

    }; // class Hackflight

} // namespace hf
