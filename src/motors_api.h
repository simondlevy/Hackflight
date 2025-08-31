/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#pragma once

#include <stdbool.h>

int   motorsGetRatio(uint32_t id);
void  motorsInit(void);
bool  motorsTest(void);
void  motorsSetRatios(const uint16_t ratios[]);

#ifdef __cplusplus
extern "C" {
#endif

    // can be called from nvic.c
    void  motorsStop();

#ifdef __cplusplus
}
#endif
