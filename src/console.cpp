/**
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include <stdarg.h>
#include <stdio.h>

#include <console.h>

int consolePrintf(const char * fmt, ...)
{
    va_list ap = {};

    va_start(ap, fmt);

    char buf[256] = {};

    int len = vsprintf(buf, fmt, ap);

    Serial.print(buf);

    va_end(ap);

    return len;
}
