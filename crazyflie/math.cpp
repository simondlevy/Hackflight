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

#include <arm_math.h>

void KalmanFilter::device_mat_trans(const matrix_t * pSrc, matrix_t * pDst)
{
  arm_mat_trans_f32((arm_matrix_instance_f32 *)pSrc,
          (arm_matrix_instance_f32 *)pDst);
}

void KalmanFilter::device_mat_mult(
        const matrix_t * pSrcA, const matrix_t * pSrcB,
        matrix_t * pDst) 
{
  arm_mat_mult_f32((arm_matrix_instance_f32 *)pSrcA, (arm_matrix_instance_f32 *)pSrcB,
          (arm_matrix_instance_f32 *)pDst);
}

float KalmanFilter::device_cos(const float x)
{
    return arm_cos_f32(x);
}

float KalmanFilter::device_sin(const float x)
{
    return arm_sin_f32(x);
}

float KalmanFilter::device_sqrt(const float32_t in) 
{
  float pOut = 0;
  arm_sqrt_f32(in, &pOut);
  return pOut;
}

/*
static inline void mat_scale(const arm_matrix_instance_f32 * pSrcA,
        float32_t scale, arm_matrix_instance_f32 * pDst) 
{
  arm_mat_scale_f32(pSrcA, scale, pDst);
}*/
