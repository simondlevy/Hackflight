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

            rft::DofPid _xpid;
            rft::DofPid _ypid;

            bool inband(float demand) 
            {
                return fabs(demand) < STICK_DEADBAND;
            }

        protected:

            virtual void modifyDemands(State * state, float * demands) override
            {
                // Get state vector
                float * x = state->x;

                // Run controller only if roll and pitch are small
                if (inband(demands[DEMANDS_ROLL]) && inband(demands[DEMANDS_PITCH])) {

                    // Get heading
                    float psi = x[State::PSI];

                    // Convert lateral velocity from world coordinates to body coordinates

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = x[State::DX];
                    float dy = x[State::DY];

                    float vfwd = cpsi * dx + spsi * dy;
                    float vrgt = cpsi * dy - spsi * dx;

                    demands[DEMANDS_ROLL] = _ypid.compute(0, vrgt);
                    demands[DEMANDS_PITCH] = _xpid.compute(0, vfwd);
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
