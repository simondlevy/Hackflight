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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <system.h>

#include "platform.h"
#include "atomic.h"
#include "io.h"
#include "light_led.h"
#include "nvic.h"
#include "resource.h"
#include "systemdev.h"

// See "ARM CoreSight Architecture Specification"
// B2.3.10  "LSR and LAR, Software Lock Status Register and Software Lock Access Register"
// "E1.2.11  LAR, Lock Access Register"

#define DWT_LAR_UNLOCK_VALUE 0xC5ACCE55

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;
static uint32_t cpuClockFrequency = 0;

void systemCycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    cpuClockFrequency = clocks.SYSCLK_Frequency;
    usTicks = cpuClockFrequency / 1000000;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
    *(DWTLAR) = DWT_LAR_UNLOCK_VALUE;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// SysTick

static volatile int sysTickPending = 0;

void SysTick_Handler(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sysTickUptime++;
        sysTickValStamp = SysTick->VAL;
        sysTickPending = 0;
        (void)(SysTick->CTRL);
    }
}

// Return system uptime in microseconds (rollover in 70minutes)

uint32_t microsISR(void)
{
    register uint32_t ms, pending, cycle_cnt;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        cycle_cnt = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            // Update pending.
            // Record it for multiple calls within the same rollover period
            // (Will be cleared when serviced).
            // Note that multiple rollovers are not considered.

            sysTickPending = 1;

            // Read VAL again to ensure the value is read after the rollover.

            cycle_cnt = SysTick->VAL;
        }

        ms = sysTickUptime;
        pending = sysTickPending;
    }

    return ((ms + pending) * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;

    // Call microsISR() in interrupt and elevated (non-zero) BASEPRI context

    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return microsISR();
    }

    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);

    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

uint32_t systemGetCycleCounter(void)
{
    return DWT->CYCCNT;
}

uint32_t systemClockMicrosToCycles(uint32_t micros)
{
    return micros * usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {

        ledSet(false);

        while (count--) {
            ledToggle();
            delay(duration);
            ledToggle();
            delay(duration);
        }
    }
}

void systemIndicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    while (codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);

        delay(WARNING_PAUSE_DURATION_MS);

        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);

        delay(1000);
    }
}

void systemFailureMode(failureMode_e mode)
{
    systemIndicateFailure(mode, 10);

    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
}

void systemInitialiseMemorySections(void)
{
    extern uint8_t _sfastram_data;
    extern uint8_t _efastram_data;
    extern uint8_t _sfastram_idata;
    memcpy(&_sfastram_data, &_sfastram_idata, (size_t) (&_efastram_data - &_sfastram_data));
}

static void unusedPinInit(IO_t io)
{
    if (IOGetOwner(io) == OWNER_FREE) {
        IOConfigGPIO(io, IOCFG_IPU);
    }
}

void systemInitUnusedPins(void)
{
    IOTraversePins(unusedPinInit);
}

void systemReboot(void)
{
    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
}

void delayMillis(uint32_t ms)
{
    delay(ms);
}

uint32_t timeMicros(void)
{
    return micros();
}

uint32_t timeMillis(void)
{
    return millis();
}
