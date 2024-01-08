#include <stdio.h>
#include <stdarg.h>

#include <crossplatform.h>

int consolePrintf(const char * fmt, ...)
{
  va_list ap = {};

  va_start(ap, fmt);

  char s[256] = {};

  auto len = vsprintf(s, fmt, ap);

  va_end(ap);

  puts(s);

  return len;
}




