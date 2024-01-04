/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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

enum {
    MOTOR_M1,
    MOTOR_M2,
    MOTOR_M3,
    MOTOR_M4,
};

typedef enum {
    BRUSHED,
    BRUSHLESS,
} motorsDrvType;


#ifdef __cplusplus
extern "C" {
#endif

    void  motorsCheckDshot();
    int   motorsGetRatio(uint32_t id);
    void  motorsInit(void);
    bool  motorsTest(void);
    void  motorsSetRatios(const uint16_t ratios[]);
    void  motorsStop();

#ifdef __cplusplus
}
#endif
