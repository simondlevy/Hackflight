/*
   State datatype

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class State {

        public:

            bool armed;
            bool failsafe;

            virtual bool safeToArm(void) 
            {
                return true;
            }

    }; // class State

} // namespace hf
