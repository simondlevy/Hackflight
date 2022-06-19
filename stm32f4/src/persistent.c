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

/*
 * An implementation of persistent data storage utilizing RTC backup data register.
 * Retains values written across software resets and boot loader activities.
 */

#include <stdint.h>
#include "platform.h"

#include "persistent.h"
#include "systemdev.h"

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

uint32_t persistentObjectRead(persistentObjectId_e id)
{
    uint32_t value = RTC_ReadBackupRegister(id);

    return value;
}

void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    RTC_WriteBackupRegister(id, value);
}

void persistentObjectRTCEnable(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Enable Access to PWR
    PWR_BackupAccessCmd(ENABLE); // Disable backup domain protection

    // We don't need a clock source for RTC itself. Skip it.

    RTC_WriteProtectionCmd(ENABLE);  // Reset sequence
    RTC_WriteProtectionCmd(DISABLE); // Apply sequence
}

void persistentObjectInit(void)
{
    // Configure and enable RTC for backup register access

    persistentObjectRTCEnable();

    // XXX Magic value checking may be sufficient

    uint32_t wasSoftReset;

    wasSoftReset = RCC->CSR & RCC_CSR_SFTRSTF;

    if (!wasSoftReset || (persistentObjectRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
        for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            persistentObjectWrite(i, 0);
        }
        persistentObjectWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
    }
}
