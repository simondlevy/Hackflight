/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    class PositionHoldPid : public PidController {

        private:

            static constexpr float STICK_DEADBAND = 0.20;   

            bool inband(float demand) 
            {
                return fabs(demand) < STICK_DEADBAND;
            }

            // Helper class
            class VelPid : public rft::DofPid {

                public:

                    float compute(float vel) {

                        return DofPid::compute(0, vel);
                    }
            }; 

            // One PID controller for each axis (X, Y)
            VelPid _xpid;
            VelPid _ypid;

        protected:

            virtual void modifyDemands(State * state, float * demands) override
            {
                // Get state vector
                float * x = state->x;

                // Run controller only if roll and pitch are small
                if (inband(demands[DEMANDS_ROLL]) && inband(demands[DEMANDS_PITCH])) {

                    // Get heading for rotating world-coordinate velocities into body coordinates
                    float psi = x[State::PSI];

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = x[State::DX];
                    float dy = x[State::DY];

                    demands[DEMANDS_ROLL] = _ypid.compute(cpsi * dy - spsi * dx);
                    demands[DEMANDS_PITCH] = _xpid.compute(cpsi * dx + spsi * dy);
                }
            }

        public:

            PositionHoldPid(const float Kp=0.1, const float Ki=0.0, const float Kd=0.0) 
            {
                _xpid.begin(Kp, Ki, Kd);
                _ypid.begin(Kp, Ki, Kd);
            }

    };  // class PositionHoldPid

} // namespace hf
