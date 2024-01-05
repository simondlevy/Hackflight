/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 * console.c - Used to send console data to client
 */

#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <stm32f4xx.h>

#include "crtp/crtp.h"

#include "console.h"

typedef int (*putc_t)(int c);

static const char digit[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
                             'A', 'B', 'C', 'D', 'E', 'F'};

static int getIntLen (long int value)
{
  int l = 1;
  while(value > 9)
  {
    l++;
    value /= 10;
  }
  return l;
}

static int power(int a, int b)
{
  int i;
  int x = a;

  for (i = 1; i < b; i++)
  {
    x *= a;
  }

  return x;
}

static int itoa10Unsigned(putc_t putcf, unsigned long long int num)
{
  int len = 0;

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  unsigned long long int i = 1;

  while ((num / i) > 9)
  {
    i *= 10L;
  }

  do
  {
    putcf(digit[(num / i) % 10L]);
    len++;
  }
  while (i /= 10L);

  return len;
}

static int itoa10(putc_t putcf, long long int num, int precision)
{
  int len = 0;

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  long long unsigned int n = num;
  if (num < 0)
  {
    n = -num;
    putcf('-');
    len++;
  }

  int numLenght = getIntLen(num);
  if (numLenght < precision)
  {
    int fillWithZero = precision - numLenght;
    while (fillWithZero > 0)
    {
      putcf('0');
      len++;
      fillWithZero--;
    }
  }

  return itoa10Unsigned(putcf, n) + len;
}

static int itoa16(putc_t putcf, uint64_t num, int width, char padChar)
{
  int len = 0;
  bool foundFirst = false;

  for (int i = 15; i >= 0; i--)
  {
    int shift = i * 4;
    uint64_t mask = (uint64_t)0x0F << shift;
    uint64_t val = (num & mask) >> shift;

    if (val > 0)
    {
      foundFirst = true;
    }

    if (foundFirst || i < width)
    {
      if (foundFirst)
      {
        putcf(digit[val]);
      }
      else
      {
        putcf(padChar);
      }

      len++;
    }
  }

  return len;
}

static int handleLongLong(
        putc_t putcf, 
        const char** fmt
        , unsigned long long int val, 
        int width, 
        char padChar)
{
  int len = 0;

  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      // Nothing here
      break;
  }

  return len;
}

static int handleLong(
        putc_t putcf, const char** fmt, 
        unsigned long int val, 
        int width, char 
        padChar)
{
  int len = 0;

  switch(*((*fmt)++))
  {
    case 'i':
    case 'd':
      len = itoa10(putcf, (long int)val, 0);
      break;
    case 'u':
      len = itoa10Unsigned(putcf, val);
      break;
    case 'x':
    case 'X':
      len = itoa16(putcf, val, width, padChar);
      break;
    default:
      // Nothing here
      break;
  }

  return len;
}

static int evprintf(putc_t putcf, const char * fmt, va_list ap)
{
  int len=0;
  float num;
  char* str;
  int precision;
  int width;
  char padChar;

  while (*fmt)
  {
    if (*fmt == '%')
    {
      precision = 6;
      padChar = ' ';
      width = 0;

      fmt++;
      if (*fmt == '%') {
        putcf(*fmt++);
        len++;
        continue;
      }

      while ('0' == *fmt)
      {
        padChar = '0';
        fmt++;
      }

			while(isdigit((unsigned)*fmt))
			{
				width *= 10;
				width += *fmt - '0';
				fmt++;
			}

      while (!isalpha((unsigned) *fmt))
      {
        if (*fmt == '.')
        {
          fmt++;
          if (isdigit((unsigned)*fmt))
          {
            precision = *fmt - '0';
            fmt++;
          }
        }
      }
      switch (*fmt++)
      {
        case 'i':
        case 'd':
          len += itoa10(putcf, va_arg(ap, int), 0);
          break;
        case 'u':
          len += itoa10Unsigned(putcf, va_arg(ap, unsigned int));
          break;
        case 'x':
        case 'X':
          len += itoa16(putcf, va_arg(ap, unsigned int), width, padChar);
          break;
        case 'l':
          // Look ahead for ll
          if (*fmt == 'l') {
            fmt++;
            len += handleLongLong(
                    putcf, &fmt, va_arg(ap, unsigned long long int), width, padChar);
          } else {
            len += handleLong(
                    putcf, &fmt, va_arg(ap, unsigned long int), width, padChar);
          }

          break;
        case 'f':
          num = va_arg(ap, double);
          if(num<0)
          {
            putcf('-');
            num = -num;
            len++;
          }
          len += itoa10(putcf, (int)num, 0);
          putcf('.'); len++;
          len += itoa10(putcf, (num - (int)num) * power(10,precision), precision);
          break;
        case 's':
          str = va_arg(ap, char* );
          while(*str)
          {
            putcf(*str++);
            len++;
          }
          break;
        case 'c':
          putcf((char)va_arg(ap, int));
          len++;
          break;
        default:
          break;
      }
    }
    else
    {
      putcf(*fmt++);
      len++;
    }
  }
  
  return len;
}

static crtpPacket_t messageToPrint;
static bool messageSendingIsPending = false;
static xSemaphoreHandle synch = NULL;

static const char bufferFullMsg[] = "<F>\n";
static bool isInit;

static void addBufferFullMarker();


/**
 * Send the data to the client
 * returns TRUE if successful otherwise FALSE
 */
static bool consoleSendMessage(void)
{
  if (crtpSendPacket(&messageToPrint) == pdTRUE)
  {
    messageToPrint.size = 0;
    messageSendingIsPending = false;
  }
  else
  {
    return false;
  }

  return true;
}

static int consolePutcharFromISR(int ch) 
{
  BaseType_t higherPriorityTaskWoken;

  if (xSemaphoreTakeFromISR(synch, &higherPriorityTaskWoken) == pdTRUE) {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
    {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    xSemaphoreGiveFromISR(synch, &higherPriorityTaskWoken);
  }

  return ch;
}

static int consolePutchar(int ch)
{
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (!isInit) {
    return 0;
  }

  if (isInInterrupt) {
    return consolePutcharFromISR(ch);
  }

  if (xSemaphoreTake(synch, portMAX_DELAY) == pdTRUE)
  {
    // Try to send if we already have a pending message
    if (messageSendingIsPending)
    {
      consoleSendMessage();
    }

    if (! messageSendingIsPending)
    {
      if (messageToPrint.size < CRTP_MAX_DATA_SIZE)
      {
        messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
        messageToPrint.size++;
      }

      if (ch == '\n' || messageToPrint.size >= CRTP_MAX_DATA_SIZE)
      {
        if (crtpGetFreeTxQueuePackets() == 1)
        {
          addBufferFullMarker();
        }
        messageSendingIsPending = true;
        consoleSendMessage();
      }
    }
    xSemaphoreGive(synch);
  }

  return (unsigned char)ch;
}


static int findMarkerStart()
{
  int start = messageToPrint.size;

  // If last char is new line, rewind one char since the marker contains a new line.
  if (start > 0 && messageToPrint.data[start - 1] == '\n')
  {
    start -= 1;
  }

  return start;
}

static void addBufferFullMarker()
{
  // Try to add the marker after the message if it fits in the buffer, otherwise overwrite the end of the message
  int endMarker = findMarkerStart() + sizeof(bufferFullMsg);
  if (endMarker >= (CRTP_MAX_DATA_SIZE))
  {
    endMarker = CRTP_MAX_DATA_SIZE;
  }

  int startMarker = endMarker - sizeof(bufferFullMsg);
  memcpy(&messageToPrint.data[startMarker], bufferFullMsg, sizeof(bufferFullMsg));
  messageToPrint.size = startMarker + sizeof(bufferFullMsg);
}

//////////////////////////////////////////////////////////////////////////////

void consoleInit()
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  vSemaphoreCreateBinary(synch);
  messageSendingIsPending = false;

  isInit = true;
}

bool consoleTest(void)
{
  return isInit;
}

int consolePrintf(const char * fmt, ...)
{
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = evprintf(consolePutchar, fmt, ap);
  va_end(ap);

  return len;
}


