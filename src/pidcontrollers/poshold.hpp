/*
   Positon-hold PID controller

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    // Helper class
    class _PosHoldPid {

        private:

            static constexpr float STICK_DEADBAND = 0.20;   

            float _Kp = 0;

            uint8_t _state_axis = 0;
            uint8_t _demand_axis = 0;
            int8_t _state_direction = 0;

            bool inband(float demand) 
            {
                return fabs(demand) < STICK_DEADBAND;
            }

        protected:

            _PosHoldPid(
                    const float Kp,
                    uint8_t state_axis,
                    uint8_t demand_axis,
                    uint8_t state_direction=+1) 
            {
                _Kp = Kp;
                _state_axis = state_axis;
                _demand_axis = demand_axis;
                _state_direction = state_direction;
            }

            void compute(State * state, float * demands)
            {
                // Get state vector
                float * x = state->x;

                // Run controller only if roll and pitch are small
                if (inband(demands[_demand_axis])) {

                    // Get heading for rotating world-coordinate velocities
                    // into body coordinates
                    float psi = x[State::PSI];

                    float cpsi = cos(psi);
                    float spsi = sin(psi);

                    float dx = x[State::DX];
                    float dy = x[State::DY];

                    demands[_demand_axis] = -_Kp * rotate(dx, dy, cpsi, spsi);
                }
            }

            virtual float rotate(float dx, float dy,
                    float cpsi, float spsi) = 0;

    };  // class _PosHoldPid

    class XPositionHoldPid : public PidController, protected _PosHoldPid {

        public:

            XPositionHoldPid(const float Kp=0.1)
                : _PosHoldPid(Kp, State::DX, DEMANDS_PITCH)
            {
            }

            void modifyDemands(State * state, float * demands) override
            {
                _PosHoldPid::compute(state, demands);
            }

             virtual float rotate(float dx, float dy,
                    float cpsi, float spsi) override
            {
                return cpsi * dx + spsi * dy;
            }

    }; // class XPositionHoldPid

    class YPositionHoldPid : public PidController, protected _PosHoldPid {

        public:

            YPositionHoldPid(const float Kp=0.1)
                : _PosHoldPid(Kp, State::DY, DEMANDS_ROLL)
            {
            }

            void modifyDemands(State * state, float * demands) override
            {
                _PosHoldPid::compute(state, demands);
            }

            virtual float rotate(float dx, float dy,
                    float cpsi, float spsi) override
            {
                return cpsi * dy - spsi * dx;
            }

    }; // class YPositionHoldPid

} // namespace hf
