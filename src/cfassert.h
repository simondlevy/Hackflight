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
 * cfassert.h - Assert macro
 */

#pragma once

#include <stdbool.h>

#define ASSERT(e)  if (e) ; else assertFail( (char *)#e, (char *)__FILE__, __LINE__ )

#define ASSERT_DMA_SAFE(PTR) if (((uint32_t)(PTR) >= 0x10000000) && ((uint32_t)(PTR) <=  0x1000FFFF)) assertFail( (char *)"", (char *)__FILE__, __LINE__ )

void assertFail(char *exp, char *file, int line);

