/* Code for a pushbutton switch
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

#pragma once

#include <hackflight.h>

namespace hf {

    static constexpr uint16_t kThreshold = 5;
    static constexpr uint32_t kDebounceDelayMsec = 50;

    class Debouncer {

        private:

            static constexpr uint16_t kThreshold = 5;
            static constexpr uint32_t kDebounceDelayMsec = 50;

        public:

            Debouncer() = default;

            Debouncer(const uint8_t pin) 
                : pin_(pin) {}

            auto Read() -> bool
            {
                int reading = analogRead(pin_) < kThreshold;

                if (reading != reading_) {
                    last_debounce_msec_ = millis();
                }

                if ((millis() - last_debounce_msec_) > kDebounceDelayMsec) {

                    if (reading != state_) {
                        state_ = reading;
                    }
                }

                reading_ = reading;

                return state_;
            }

        private:

            uint8_t pin_;
            uint8_t reading_;
            uint32_t last_debounce_msec_;
            uint8_t state_;
    };

    class LatchingPushbutton {

        public:

            LatchingPushbutton(const uint8_t pin) 
                : debouncer_(pin) {}

            auto Read() -> bool
            {
                return debouncer_.Read();
            }

        private:

            Debouncer debouncer_;
    };

    class IntermittentPushbutton {

        public:

            IntermittentPushbutton(const uint8_t pin) 
                : debouncer_(pin) {}

            auto Read() -> bool
            {
                const auto state = debouncer_.Read();

                if (state && !oldstate_) {
                    output_ = !output_;
                }

                oldstate_ = state;

                return output_;
            }

        private:

            Debouncer debouncer_;

            bool oldstate_;
            bool output_;
    };

}


