/*
Copyright (c) 2022 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>

#include <led.h>

class ArduinoLed : public Led {

    private:

        bool m_on;

    protected:

        virtual void devInit(void) override
        {
            pinMode(m_pin, OUTPUT);
        }

        virtual void devSet(bool on) override
        {
            digitalWrite(m_pin, on);
            m_on = on;
        }

        virtual void devToggle(void) override
        {
            m_on = !m_on;
            devSet(m_on);
        }

    public:

        ArduinoLed(uint8_t pin) 
            : Led(pin)
        {
            m_on = false;
        }

}; // class ArduinoLed 


