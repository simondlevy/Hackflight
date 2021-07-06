/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class PositionHoldPid : public PidController {


        private:

            static constexpr float MAX_POSHOLD_ANGLE_DEGREES = 5;

        protected:

            virtual void modifyDemands(State * state, float * demands) override
            {
                float * x = state->x;
                float phi = x[State::PHI];
                float theta = x[State::THETA];
                float psi = x[State::PSI];
                float cpsi = cos(psi);
                float spsi = sin(psi);
                float dx = x[State::DX];
                float dy = x[State::DY];
                float vfwd = cpsi * dx + spsi * dy;
                float vrgt = cpsi * dy - spsi * dx;

                rft::Debugger::printf("%3.0f", fabs(rft::Filter::rad2deg(phi)));
            }

        public:

            PositionHoldPid(void)
            {
            }

    };  // class PositionHoldPid

} // namespace hf
