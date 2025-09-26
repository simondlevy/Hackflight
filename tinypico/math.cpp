/*
 * Copyright (C) 2018-2021 Bitcraze AB, 2025 Simon D. Levy
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
 */

#include <kalman.hpp>

#include "xtensa_math.h"

void KalmanFilter::device_mat_trans(const matrix_t * pSrc, matrix_t * pDst)
{
}

void KalmanFilter::device_mat_mult(
        const matrix_t * pSrcA, const matrix_t * pSrcB,
        matrix_t * pDst) 
{
}

float KalmanFilter::device_cos(const float x)
{
    return 0;
}

float KalmanFilter::device_sin(const float x)
{
    return 0;
}

float KalmanFilter::device_sqrt(const float32_t in) 
{
    return 0;
}
