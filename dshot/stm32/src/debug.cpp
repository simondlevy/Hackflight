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

#include "debug.h"

// ----------------------------------------------------------------------------

putcf stdout_putf;

void debugFlush(void)
{
    while (!serialIsTransmitBufferEmpty(printfSerialPort));
}

void debugPrintf(const char *fmt, ...)
{
    void mspTriggerDebugging(void);

    va_list va;
    va_start(va, fmt);
    tfp_format(NULL, stdout_putf, fmt, va);
    va_end(va);
    mspTriggerDebugging();
}

void debugSetPort(void * p)
{
    printfSerialPort = p;
    stdout_putf = _putc;
}
