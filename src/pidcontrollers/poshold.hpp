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
                float * x = state->x;
                float phi = x[State::PSI];
                float cphi = cos(phi);
                float sphi = sin(phi);
                float dx = x[State::DX];
                float dy = x[State::DY];
                float vfwd = cphi * dx + sphi * dy;
                float vrgt = cphi * dy - sphi * dx;

                rft::Debugger::printf("%+3.2f", vfwd);
            }

        public:

            PositionHoldPid(void)
            {
            }

    };  // class PositionHoldPid

} // namespace hf
