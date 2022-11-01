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

class RealLed : public Led {

    private:

        uint8_t m_pin;
        bool m_on;
        bool m_inverted;

    protected:

        virtual void devInit(void) override
        {
            pinMode(m_pin, OUTPUT);
        }

        virtual void devSet(bool on) override
        {
            digitalWrite(m_pin, m_inverted ? on : !on);
            m_on = on;
        }

        virtual void devToggle(void) override
        {
            m_on = !m_on;
            devSet(m_on);
        }

    public:

        RealLed(uint8_t pin, bool inverted=false) 
        {
            m_pin = pin;
            m_inverted = inverted;
            m_on = false;
        }

}; // class RealLed 


