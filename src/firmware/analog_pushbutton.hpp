/* Push-button support class
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

#include <Arduino.h>

class AnalogPushButton {

    private:

        static const uint32_t kDebounceDelayMsec = 50; 
    
    public:

        AnalogPushButton(const uint8_t pin, const uint16_t threshold)
            :
                pin_(pin),
                threshold_(threshold),
                output_(true),
                button_state_(HIGH),
                last_button_state_(HIGH) {}

        auto Read() -> bool
        {
            const auto button_state =  analogRead(pin_) > 4094;

            if (button_state != last_button_state_) {
                debounce_time_msec_ = millis();
            }

            if ((millis() - debounce_time_msec_) > kDebounceDelayMsec) {

                if (button_state != button_state_) {

                    button_state_ = button_state;

                    if (button_state_ == LOW) {
                        output_ = !output_; 
                    }
                }
            }

            last_button_state_ = button_state;
            return output_;
        }

    private:

        uint8_t pin_;
        uint16_t threshold_;
        bool output_;
        uint8_t button_state_;
        uint8_t last_button_state_;
        uint32_t debounce_time_msec_;
};
