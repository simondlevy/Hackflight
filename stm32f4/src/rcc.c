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

#include "platform.h"
#include "rcc.h"

void RCC_ClockCmd(uint8_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

    switch (tag) {
    case RCC_APB2:
        RCC_APB2PeriphClockCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphClockCmd(mask, NewState);
        break;
    case RCC_AHB1:
        RCC_AHB1PeriphClockCmd(mask, NewState);
        break;
    }
}

void RCC_ResetCmd(uint8_t periphTag, FunctionalState NewState)
{
    int tag = periphTag >> 5;
    uint32_t mask = 1 << (periphTag & 0x1f);

// Peripheral reset control relies on RSTR bits are identical to ENR bits where applicable

#define __HAL_RCC_FORCE_RESET(bus, suffix, enbit) (RCC->bus ## RSTR ## suffix |= (enbit))
#define __HAL_RCC_RELEASE_RESET(bus, suffix, enbit) (RCC->bus ## RSTR ## suffix &= ~(enbit))
#define __HAL_RCC_RESET(bus, suffix, enbit, NewState) \
    if (NewState == ENABLE) {                         \
        __HAL_RCC_RELEASE_RESET(bus, suffix, enbit);  \
    } else {                                          \
        __HAL_RCC_FORCE_RESET(bus, suffix, enbit);    \
    }

    switch (tag) {
    case RCC_APB2:
        RCC_APB2PeriphResetCmd(mask, NewState);
        break;
    case RCC_APB1:
        RCC_APB1PeriphResetCmd(mask, NewState);
        break;
    case RCC_AHB1:
        RCC_AHB1PeriphResetCmd(mask, NewState);
        break;
    }
}
