/*
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

#include <led.h>

#include "io.h"

class Stm32F4Led : public Led {

    private:

        IO_t m_led;

    protected:

        virtual void devInit(void) override
        {
            m_led = IOGetByTag(m_pin);
            IOInit(m_led, OWNER_LED, RESOURCE_INDEX(0));
            IOConfigGPIO(m_led, IOCFG_OUT_PP);
        }

        virtual void devSet(bool on) override
        {
            IOWrite(m_led, !on);
        }

        virtual void devToggle(void) override
        {
            IOToggle(m_led);
        }

    public:

        Stm32F4Led(uint8_t pin) 
            : Led(pin)
        {
        }

}; // class Stm32F4Led 


