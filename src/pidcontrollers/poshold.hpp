/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class PositionHoldPid : public PidController {


        private:

            static constexpr float MAX_ANGLE_DEGREES = 5;

            float _maxAngle = 0;

            bool safeAngle(float angle) {
                return abs(angle) < _maxAngle;
            }

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

        public:

            PositionHoldPid(void)
            {
                _maxAngle = rft::Filter::deg2rad(MAX_ANGLE_DEGREES);
            }

    };  // class PositionHoldPid

} // namespace hf
