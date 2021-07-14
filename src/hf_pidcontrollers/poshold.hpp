/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class PositionHoldPid : public PidController {

        private:

            // Constants set in constructor ----------------------------

            float _Kp = 0;
            float _stickDeadband = 0.2;   

            // ----------------------------------------------------------

            // Helper
            bool inband(float demand) 
            {
                return fabs(demand) < _stickDeadband;
            }

        protected:

            void modifyDemands(State * state, float * demands) override
            {
                // Get state vector
                float * x = state->x;

                // Run controller only if roll and pitch are small
                if (inband(demands[DEMANDS_PITCH]) && inband(demands[DEMANDS_ROLL])) {

                    // Get heading for rotating world-coordinate velocities
                    // into body coordinates
                    float psi = x[State::PSI];

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = x[State::DX];
                    float dy = x[State::DY];

                    demands[DEMANDS_ROLL] = -_Kp * (cpsi * dy - spsi * dx);
                    demands[DEMANDS_PITCH] = -_Kp * (cpsi * dx + spsi * dy);
                }
            }

        public:

            PositionHoldPid(float Kp=0.1, float stickDeadband=0.2)
            {
                _Kp = Kp;
                _stickDeadband = stickDeadband;
            }

    }; // class PositionHoldPid

} // namespace hf
