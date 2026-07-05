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

class PushButton {

    public:

        PushButton(const uint8_t pin) : pin_(pin) { }

        void begin()
        {
            pinMode(pin_, INPUT);
        }

        auto read() -> bool
        {
            const auto button_is_down = digitalRead(pin_);

            if (button_is_down && !button_was_down_) {
                value_ = !value_;
            }
            button_was_down_ = button_is_down;

            return value_;
        }

    private:

        uint8_t pin_;
        bool value_;
        bool button_was_down_;
};
