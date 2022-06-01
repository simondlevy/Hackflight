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
#include "light_led.h"

static IO_t leds[STATUS_LED_NUMBER];

void ledInit(void)
{
    leds[0] = IOGetByTag(37);
    IOInit(leds[0], OWNER_LED, RESOURCE_INDEX(0));
    IOConfigGPIO(leds[0], IOCFG_OUT_PP);

    leds[1] = IO_NONE;
    leds[2] = IO_NONE;
}

void ledToggle(void)
{
    IOToggle(leds[0]);
}

void ledSet(bool on)
{
    IOWrite(leds[0], !on);
}
