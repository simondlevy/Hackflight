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


#include <serial.h>

typedef void (*putcf) (void *, char);

static void * printfSerialPort;

static int a2d(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    else
        return -1;
}

static char a2i(char ch, const char **src, int base, int *nump)
{
    const char *p = *src;
    int num = 0;
    int digit;
    while ((digit = a2d(ch)) >= 0) {
        if (digit > base)
            break;
        num = num * base + digit;
        ch = *p++;
    }
    *src = p;
    *nump = num;
    return ch;
}

static void uli2a(unsigned long int num, unsigned int base, int uc, char *bf)
{
    unsigned int d = 1;

    while (num / d >= base)
        d *= base;

    while (d != 0) {
        int dgt = num / d;
        *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);

        // Next digit
        num %= d;
        d /= base;
    }
    *bf = 0;
}

static void li2a(long num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    uli2a(num, 10, 0, bf);
}

static void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
    unsigned int d = 1;

    while (num / d >= base)
        d *= base;

    while (d != 0) {
        int dgt = num / d;
        *bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);

        // Next digit
        num %= d;
        d /= base;
    }
    *bf = 0;
}

static void i2a(int num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    ui2a(num, 10, 0, bf);
}

// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void *putp, putcf putf, int n, char z, char *bf)
{
    int written = 0;
    char fc = z ? '0' : ' ';
    char ch;
    char *p = bf;
    while (*p++ && n > 0)
        n--;
    while (n-- > 0) {
        putf(putp, fc); written++;
    }
    while ((ch = *bf++)) {
        putf(putp, ch); written++;
    }
    return written;
}

// retrun number of bytes written
static int tfp_format(void *putp, putcf putf, const char *fmt, va_list va)
{
    char bf[12];
    int written = 0;
    char ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putf(putp, ch); written++;
        } else {
            char lz = 0;
            char lng = 0;
            int w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
            if (ch == 'l') {
                ch = *(fmt++);
                lng = 1;
            }
            switch (ch) {
                case 0:
                    goto abort;
                case 'u':{
                             if (lng)
                                 uli2a(va_arg(va, unsigned long int), 10, 0, bf);
                             else
                                 ui2a(va_arg(va, unsigned int), 10, 0, bf);
                             written += putchw(putp, putf, w, lz, bf);
                             break;
                         }
                case 'd':{
                             if (lng)
                                 li2a(va_arg(va, unsigned long int), bf);
                             else
                                 i2a(va_arg(va, int), bf);
                             written += putchw(putp, putf, w, lz, bf);
                             break;
                         }
                case 'x':
                case 'X':
                         if (lng)
                             uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
                         else
                             ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
                         written += putchw(putp, putf, w, lz, bf);
                         break;
                case 'c':
                         putf(putp, (char) (va_arg(va, int))); written++;
                         break;
                case 's':
                         written += putchw(putp, putf, w, 0, va_arg(va, char *));
                         break;
                case '%':
                         putf(putp, ch); written++;
                         break;
                case 'n':
                         *va_arg(va, int*) = written;
                         break;
                default:
                         break;
            }
        }
    }
abort:
    return written;
}

static void _putc(void *p, char c)
{
    (void)p;
    serialWrite(printfSerialPort, c);
}

// ----------------------------------------------------------------------------

putcf stdout_putf;

void serialDebugFlush(void)
{
    while (!serialIsTransmitBufferEmpty(printfSerialPort));
}

void serialDebugPrintf(const char *fmt, ...)
{
    //void mspTriggerDebugging(void);

    va_list va;
    va_start(va, fmt);
    tfp_format(NULL, stdout_putf, fmt, va);
    va_end(va);
    //mspTriggerDebugging();
}

void serialDebugSetPort(void * p)
{
    printfSerialPort = p;
    stdout_putf = _putc;
}
