/* Utility for blinking LEDs
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <firmware/timer.hpp>

namespace hf {

    class BlinkTimer {

        private:

            static constexpr float kFreqHz = 2;

        public:

            BlinkTimer(const Timer & timer, const bool on)
                : timer_(timer), on_(on) {}

            BlinkTimer() = default;

            BlinkTimer(const BlinkTimer & bt) = default;

            bool On() {

                timer_ = Timer::Update(timer_, kFreqHz, millis());

                if (Timer::IsReady(timer_)) {
                    on_ = !on_;
                }

                return on_;
            }

            static BlinkTimer Update(
                    const BlinkTimer & bt,
                    const uint32_t msec)
            {
                const auto timer = Timer::Update(bt.timer_, kFreqHz, msec);

                const auto on = Timer::IsReady(timer) ? !bt.on_ : bt.on_;

                return BlinkTimer(timer, on);
            }

            static auto IsOn(const BlinkTimer & bt) 
            {
                return bt.on_;
            }

        private:

            Timer timer_;
            bool on_;

    }; // class BlinkTimer

} // namespace hf
