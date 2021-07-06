/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class PositionHoldPid : public PidController {


        protected:

            virtual void modifyDemands(State * state, float * demands) override
            {
                rft::Debugger::printf("DX=%+3.2f    DY=%+3.2f", state->x[State::DX], state->x[State::DY]);
            }

        public:

            PositionHoldPid(void)
            {
            }

    };  // class PositionHoldPid

} // namespace hf
