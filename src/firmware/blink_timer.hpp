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

        public:

            BlinkTimer(const float freq_hz = 2)
                : timer_(freq_hz), on_(false) {}

            bool On() {

                if (timer_.Ready()) {
                    on_ = !on_;
                }

                return on_;
            }

        private:

            Timer timer_;
            bool on_;

    }; // class BlinkTimer

} // namespace hf
