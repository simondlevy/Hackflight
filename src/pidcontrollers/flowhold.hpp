/*
   Positon-hold PID controller using optical flow (body-frame velocity)

   Copyright (c) 2018 Juan Gallostra and Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

#include "datatypes.hpp"
#include "pidcontroller.hpp"

namespace hf {

    class FlowHoldPid : public PidController {

        friend class Hackflight;

        private: 

            // Helper class
            class _VelocityPid : public Pid {

                private:

                    // Arbitrary constants
                    static constexpr float VEL_WINDUP_MAX   = 0.40f;
                    static constexpr float PILOT_VELXY_MAX  = 2.5f; // http://ardupilot.org/copter/docs/altholdmode.html

                    bool _inBand = false;
                    bool _inBandPrev = false;

                public:

                    void init(float Kp, float Ki)
                    {
                        Pid::init(Kp, Ki, 0);

                        _inBand = false;
                        _inBandPrev = false;
                    }

                    float compute(float demand, float velocity)
                    {
                        // Is throttle stick in deadband?
                        _inBand = fabs(demand) < STICK_DEADBAND; 

                        // Reset controller when moving into deadband
                        if (_inBand && !_inBandPrev) {
                            reset();
                        }
                        _inBandPrev = _inBand;

                        // Target velocity is zero inside deadband, scaled constant outside
                        float targetVelocity = _inBand ? 0 : 2 * demand * PILOT_VELXY_MAX;

                        // Run velocity PID controller to get correction
                        return Pid::compute(targetVelocity, velocity);
                    }

            }; // _VelocityPid

            _VelocityPid _rollPid;
            _VelocityPid _pitchPid;

        protected:

            void modifyDemands(state_t & state, demands_t & demands, float currentTime)
            {
                demands.roll  = _rollPid.compute(demands.roll,  state.bodyVel[1]);
                demands.pitch = _rollPid.compute(demands.pitch, state.bodyVel[0]);
            }

            virtual bool shouldFlashLed(void) override 
            {
                return true;
            }

        public:

            FlowHoldPid(const float Kp, float Ki)
            {
                _rollPid.init(Kp, Ki);
                _pitchPid.init(Kp, Ki);
            }

    };  // class FlowHoldPid

} // namespace hf
