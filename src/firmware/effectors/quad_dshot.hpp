/*
   Quadcopter motor mixer for DSHOT motors

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>

// Third-party libraries
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/fc.hpp>
#include <mixers/quadx.hpp>

namespace hf {

    class QuadDshot {

        private:

            std::vector<uint8_t> kMotorPins = {2, 3, 4, 5};

        public:

            void Begin()
            {
                motors_.begin();
            }

            void Run(FlightController & fc, const Setpoint & setpoint)
            {
                mixer_ = QuadXMixer::Run(setpoint);

                const auto motorvals = GetMotorValues();

                // Run motors if safe
                if (fc.IsSafeToFly()) {
                    motors_.run(fc.IsArmed(), motorvals);
                }

                fc.SendTelemetry(
                        setpoint, kMspQuadrotorTelemetry, motorvals, 4);
            }

            auto GetMotorValues() -> float *
            {

                motorvals_[0] = mixer_.rr_cw;
                motorvals_[1] = mixer_.rf_ccw;
                motorvals_[2] = mixer_.lr_ccw;
                motorvals_[3] = mixer_.lf_cw;

                return motorvals_;
            }

        private:

            DshotTeensy4 motors_ = DshotTeensy4(kMotorPins);

            hf::QuadXMixer mixer_;

           float motorvals_[4];

    }; // class QuadDshot

} // namespace hf
