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

    class GroundedAnalogButton {

        private:

            static constexpr uint16_t kThreshold = 10;
            static constexpr uint32_t kDebounceDelayMsec = 50;

        public:

            GroundedAnalogButton(const uint8_t pin) 
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

                // save the reading. Next time through the loop, it'll be the reading_:
                reading_ = reading;

                return state_;
            }

        private:

            uint8_t pin_;
            uint8_t reading_;
            uint32_t last_debounce_msec_;
            uint8_t state_;

    };


    class PushbuttonSwitch {

        public:

            PushbuttonSwitch(const uint8_t pin) 
                : pin_(pin), state_(false), prev_(false) {}

            void Begin()
            {
                pinMode(pin_, INPUT);
                prev_ = digitalRead(pin_);
            }

            auto Read() -> bool
            {
                const auto curr = digitalRead(pin_);

                if (Test(curr, prev_)) {
                    state_  = !state_;
                }
                prev_ = curr;

                return state_;
            }

        private:

            virtual bool Test(const bool curr, const bool prev) = 0;

            uint8_t pin_;
            bool state_;
            bool prev_;
    };
}


