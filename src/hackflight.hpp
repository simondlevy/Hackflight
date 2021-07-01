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
            SerialTask _gcsTask;

            // Serial timer task for telemetry on secondary port (Serial1, Serial2, ...)
            SerialTask _telemetryTask = SerialTask(true);

        public:

            Hackflight(rft::Board * board, Receiver * receiver, rft::Actuator * actuator)
                : HackflightPure(board, receiver, actuator)
            {  
            }
            void begin()
            {  
                HackflightPure::begin();

                // Start serial tasks
                _gcsTask.begin(_board, &_state, _olc, _actuator);
                _telemetryTask.begin(_board, &_state, _olc, _actuator);

            }

            void update(void)
            {
                HackflightPure::update();

                // Update serial tasks
                _gcsTask.update();
                _telemetryTask.update();
            }

    }; // class Hackflight

} // namespace hf
