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
#include <mixers/bfquadx.hpp>

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

                static float motorvals[4] = {
                    mixer_.rr_cw,
                    mixer_.rf_ccw,
                    mixer_.lr_ccw,
                    mixer_.lf_cw,
                };

                return motorvals;
            }

        private:

            DshotTeensy4 motors_ = DshotTeensy4(kMotorPins);

            hf::Mixer mixer_;


    }; // class QuadDshot

} // namespace hf
