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

            rft::DofPid _pid;

        protected:

            virtual void modifyDemands(State * state, float * demands) override
            {
                // Get state vector
                float * x = state->x;

                // Run controller only if roll and pitch are small
                //if (safeAngle(x[State::PHI]) && safeAngle(x[State::THETA])) {
                if (fabs(demands[DEMANDS_ROLL]) < 0.2) {

                    // Get heading
                    float psi = x[State::PSI];

                    // Convert lateral velocity from world coordinates to body coordinates

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = x[State::DX];
                    float dy = x[State::DY];

                    float vfwd = cpsi * dx + spsi * dy;
                    float vrgt = cpsi * dy - spsi * dx;

                    demands[DEMANDS_ROLL] = _pid.compute(0, vrgt);
                }
                else {
                    //rft::Debugger::printf("NO");
                }
            }

        public:

            PositionHoldPid(const float Kp=0.1, const float Ki=0.0, const float Kd=0.0) 
            {
                _maxAngle = rft::Filter::deg2rad(MAX_ANGLE_DEGREES);

                _pid.begin(Kp, Ki, Kd);
            }

    };  // class PositionHoldPid

} // namespace hf
