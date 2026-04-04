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

#include <arm_math.h>

#include <hackflight.h>
#include <firmware/ekf.hpp>
using namespace hf;

static void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{
    arm_mat_trans_f32((arm_matrix_instance_f32 *)pSrc,
            (arm_matrix_instance_f32 *)pDst);
}

static void mat_mult(
        const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB,
        arm_matrix_instance_f32 * pDst) 
{
    arm_mat_mult_f32((arm_matrix_instance_f32 *)pSrcA, (arm_matrix_instance_f32 *)pSrcB,
            (arm_matrix_instance_f32 *)pDst);
}

void EKF::device_predict(const float F[STATE_DIM][STATE_DIM],
        float P[STATE_DIM][STATE_DIM])
{
    static __attribute__((aligned(4))) arm_matrix_instance_f32 Fm = { 
        STATE_DIM, STATE_DIM, (float *)F
    };

    static float tmpNN1d[STATE_DIM * STATE_DIM];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { 
        STATE_DIM, STATE_DIM, tmpNN1d
    };

    static float tmpNN2d[STATE_DIM * STATE_DIM];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { 
        STATE_DIM, STATE_DIM, tmpNN2d
    };

    arm_matrix_instance_f32 _p_m = {};
    _p_m.numRows = STATE_DIM;
    _p_m.numCols = STATE_DIM;
    _p_m.pData = (float*)P;

    mat_mult(&Fm, &_p_m, &tmpNN1m); // F P
    mat_trans(&Fm, &tmpNN2m); // F'

    mat_mult(&tmpNN1m, &tmpNN2m, &_p_m); // F P F'
}

void EKF::device_update_with_scalar(
        const float P[STATE_DIM][STATE_DIM],
        const float * h,
        const float error,
        const float R,
        float x[STATE_DIM],
        float G[STATE_DIM])
{
    arm_matrix_instance_f32 Hm = {1, STATE_DIM, (float *)h};

    // The Kalman gain as a column vector
    static arm_matrix_instance_f32 Gm = {STATE_DIM, 1, (float *)G};

    // Temporary matrices for the covariance updates
    static float tmpNN1d[STATE_DIM * STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {
        STATE_DIM, STATE_DIM, tmpNN1d
    };

    static float tmpNN2d[STATE_DIM * STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = {
        STATE_DIM, STATE_DIM, tmpNN2d
    };

    static float tmpNN3d[STATE_DIM * STATE_DIM];
    static arm_matrix_instance_f32 tmpNN3m = {
        STATE_DIM, STATE_DIM, tmpNN3d
    };

    static float HTd[STATE_DIM * 1];
    static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, HTd};

    static float PHTd[STATE_DIM * 1];
    static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHTd};

    arm_matrix_instance_f32 _p_m = {};
    _p_m.numRows = STATE_DIM;
    _p_m.numCols = STATE_DIM;
    _p_m.pData = (float*)P;

    mat_trans(&Hm, &HTm);
    mat_mult(&_p_m, &HTm, &PHTm); // PH'
    float HPHR = R; // HPH' + R
    for (int i=0; i<STATE_DIM; i++) { 
        // Add the element of HPH' to the above
        // this obviously only works if the update is scalar (as in this function)
        HPHR += Hm.pData[i]*PHTd[i]; 
    }

    // Calculate the Kalman gain and perform the state update
    for (int i=0; i<STATE_DIM; i++) {
        G[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
        x[i] = x[i] + G[i] * error; // state update
    }

    mat_mult(&Gm, &Hm, &tmpNN1m); // GH
    for (int i=0; i<STATE_DIM; i++) { 
        tmpNN1d[STATE_DIM*i+i] -= 1; 
    } // GH - I
    mat_trans(&tmpNN1m, &tmpNN2m); // (GH - I)'
    mat_mult(&tmpNN1m, &_p_m, &tmpNN3m); // (GH - I)*P
    mat_mult(&tmpNN3m, &tmpNN2m, &_p_m); // (GH - I)*P*(GH - I)'
}


float EKF::device_cos(const float x)
{
    return arm_cos_f32(x);
}

float EKF::device_sin(const float x)
{
    return arm_sin_f32(x);
}

float EKF::device_sqrt(const float32_t in) 
{
    float pOut = 0;
    arm_sqrt_f32(in, &pOut);
    return pOut;
}
