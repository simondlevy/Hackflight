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

            void modifyDemands(state_t & state, demands_t & demands) override
            {
                bool ready = inband(demands.pitch) && inband(demands.roll);

                // Get heading for rotating world-coordinate velocities
                // into body coordinates
                float psi = state.psi;

                float cpsi = cos(psi);
                float spsi = sin(psi);

                float dx = state.dx;
                float dy = state.dy;

                demands.roll  = ready ? -_Kp * (cpsi * dy - spsi * dx) : demands.roll;
                demands.pitch = ready ? -_Kp * (cpsi * dx + spsi * dy) : demands.pitch;
            }

        public:

            PositionHoldPid(float Kp=0.1, float stickDeadband=0.2)
            {
                _Kp = Kp;
                _stickDeadband = stickDeadband;
            }

    }; // class PositionHoldPid

} // namespace hf
