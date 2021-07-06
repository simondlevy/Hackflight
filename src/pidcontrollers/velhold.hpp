/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class PositionHoldPid : public PidController {


        private:

            // Arbitrary constants: for details see http://ardupilot.org/copter/docs/loiter-mode.html
            static constexpr float PILOT_VELZ_MAX  = 2.5f;
            static constexpr float STICK_DEADBAND = 0.10;   

        protected:

            virtual void modifyDemands(State * state, float * demands) override
            {
                // Get state vector
                float * x = state->x;

                // Run controller only if roll and pitch are small
                if (safeAngle(x[State::PHI]) && safeAngle(x[State::THETA])) {

                    // Get heading
                    float psi = x[State::PSI];

                    // Convert lateral velocity from world coordinates to body coordinates

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = x[State::DX];
                    float dy = x[State::DY];

                    float vfwd = cpsi * dx + spsi * dy;
                    float vrgt = cpsi * dy - spsi * dx;

                    rft::Debugger::printf("YES");
                }
                else {
                    rft::Debugger::printf("NO");
                }
            }

            PositionHoldPid(void)
            {
            }

    };  // class PositionHoldPid

} // namespace hf
