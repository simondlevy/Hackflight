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

#include "led.h"

#include "io.h"

static IO_t led;

void ledDevInit(uint8_t pin)
{
    led = IOGetByTag(pin);
    IOInit(led, OWNER_LED, RESOURCE_INDEX(0));
    IOConfigGPIO(led, IOCFG_OUT_PP);
}

void ledDevSet(bool on)
{
    IOWrite(led, !on);
}

void ledDevToggle(void)
{
    IOToggle(led);
}


