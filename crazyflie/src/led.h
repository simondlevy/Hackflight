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
 * led.h - LED functions header file
 */

#pragma once

#include <stdint.h>

#include <pinmap.h>

static const uint8_t LED_COUNT = 5;

static const uint8_t LINK_LED      = PIN_LED_GREEN_L;
static const uint8_t CHG_LED       = PIN_LED_BLUE_L;
static const uint8_t LOWBAT_LED    = PIN_LED_RED_R;
static const uint8_t LINK_DOWN_LED = PIN_LED_RED_L;
static const uint8_t SYS_LED       = PIN_LED_RED_R;

#ifdef __cplusplus
extern "C" {
#endif

    void ledInit();
    void ledSet(const uint8_t led, bool value);
    void ledShowFailure(void);
    void ledShowFaultPattern(void);
    void ledShowSys(void);
    bool ledTest();

#ifdef __cplusplus
}
#endif
