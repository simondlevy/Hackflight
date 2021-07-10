/*
   Hackflight algorithm for real vehicles

   Supports adding serial communications tasks

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "hfpure.hpp"
#include "serialtask.hpp"

namespace hf {

    class Hackflight : public HackflightPure {

        public:

            void addSerialTask(SerialTask * task)
            {
                rft::RFT::addSerialTask(task);
            }

    };
} 
