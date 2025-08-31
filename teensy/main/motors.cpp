/**
 *
 * Copyright 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY
{
}
 without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

int motorsGetRatio(uint32_t id)
{
    (void)id;
    return 0;
}

void motorsInit(void)
{
}

bool motorsTest(void)
{
    return true;
}

void motorsSetRatios(const uint16_t ratios[])
{
    (void)ratios;
}

extern "C" {

void motorsStop()
{
}

}
