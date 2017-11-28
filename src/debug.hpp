/*
   debug.hpp : Cross-platform serial debugging support for Hackflight

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdarg.h>

// ARM Arduino (Ladybug)
#if defined(__arm__)  
static void _puts(char * buf) { Serial.print(buf); }
static void _vsprintf(char * buf, const char * fmt, va_list ap) { vsprintf(buf, fmt, ap); }

#elif defined(_WIN32) 
// Windows
// Need to include windows.h to declare OutputDebugStringA(), but causes compiler warnings,
// so we wrap this one include in a no-warnings pragma.
// See: https://stackoverflow.com/questions/4001736/whats-up-with-the-thousands-of-warnings-in-standard-headers-in-msvc-wall
#pragma warning(push, 0) 
#include <windows.h>
#pragma warning(pop)
#include <varargs.h>
#if defined(_CONSOLE)
static void _puts(char * buf) { printf("%s", buf); }
#else
static void _puts(char * buf) { OutputDebugStringA(buf); }
#endif
static void _vsprintf(char * buf, const char * fmt, va_list ap) { vsprintf_s(buf, _vscprintf(fmt,ap)+ 1, fmt, ap); }

// Unix
#else                 
#include <stdio.h>
static void _puts(char * buf) { printf("%s", buf); }
static void _vsprintf(char * buf, const char * fmt, va_list ap) { vsprintf(buf, fmt, ap); }
#endif

namespace hf {

    class Debug {

        public:

            static void printf(const char * fmt, ...)
            {
                va_list ap;
                va_start(ap, fmt);
                char buf[200];
                _vsprintf(buf, fmt, ap);
                _puts(buf);
                va_end(ap);
            }

    }; // class Debug

} // namespace hf


