#include <stddef.h>
#include <stdint.h>

#include "arduino/analog.h"
#include "arduino/digital.h"
#include "arduino/SPI.h"
#include "arduino/time.h"

#if !defined(__main)
extern
#endif
SPIClass SPI;
