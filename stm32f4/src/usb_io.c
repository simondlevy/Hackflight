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

#include "platform.h"
#include "systemdev.h"

#include "io.h"
#include <time.h>
#include "usb_io.h"

static IO_t usbDetectPin;

void usbCableDetectInit(void)
{
    usbDetectPin = IOGetByTag(IO_TAG(NONE));

    IOInit(usbDetectPin, OWNER_USB_DETECT, 0);
    IOConfigGPIO(usbDetectPin, IOCFG_IPD);
}

void usbGenerateDisconnectPulse(void)
{
    /* Pull down PA12 to create USB disconnect pulse */
    IO_t usbPin = IOGetByTag(IO_TAG(PA12));
    IOConfigGPIO(usbPin, IOCFG_OUT_OD);

    IOLo(usbPin);

    delay(200);

    IOHi(usbPin);
}
