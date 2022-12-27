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

#if defined(STM32F4) ||  defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx)

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include "stm32f4xx.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

    void stm32_startCycleCounter(void)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

        __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
        *(DWTLAR) = 0xC5ACCE55;

        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t systemGetCycleCounter(void)
    {
        return DWT->CYCCNT;
    }

#if defined(__cplusplus)
}
#endif

#endif
