/*
 * This header helps us move toward using the Arduino API.  
 */

// NB: Putting #pragma once here will cause errors

#pragma once

#include <hal/digital.h>
#include <hal/SPI.h>

#if !defined(__main)
extern
#endif
SPIClass SPI;
