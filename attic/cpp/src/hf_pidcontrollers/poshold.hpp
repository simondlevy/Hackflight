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

            void modifyDemands(float * state, float * demands) override
            {
                // Run controller only if roll and pitch are small
                if (inband(demands[DEMANDS_PITCH]) && inband(demands[DEMANDS_ROLL])) {

                    // Get heading for rotating world-coordinate velocities
                    // into body coordinates
                    float psi = state[State::PSI];

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = state[State::DX];
                    float dy = state[State::DY];

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
