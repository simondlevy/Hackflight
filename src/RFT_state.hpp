/*
   State datatype

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

namespace rft {

    class State {

        friend class RFTPure;
        friend class ClosedLoopTask;
        friend class SerialTask;

        protected:

            bool armed = false;
            bool failsafe = false;

            State(bool start_armed=false)
            {
                armed = start_armed;
            }

            virtual bool safeToArm(void) = 0;

    }; // class State

} // namespace rft
