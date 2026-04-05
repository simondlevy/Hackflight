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
#include <firmware/ekf/ekf.hpp>
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

//////////////////////////////////////////////////////////////////////////////

// C = A * B
void EKF::device_mat_mult(
        const float A[STATE_DIM][STATE_DIM],
        const float B[STATE_DIM][STATE_DIM],
        float C[STATE_DIM][STATE_DIM])

{
    static __attribute__((aligned(4))) arm_matrix_instance_f32 _A = { 
        STATE_DIM, STATE_DIM, (float *)A
    };

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _B = { 
        STATE_DIM, STATE_DIM, (float *)B
    };

    arm_matrix_instance_f32 _C = {};
    _C.numRows = STATE_DIM;
    _C.numCols = STATE_DIM;
    _C.pData = (float*)C;

    arm_mat_mult_f32(
            (arm_matrix_instance_f32 *)&_A,
            (arm_matrix_instance_f32 *)&_B,
            (arm_matrix_instance_f32 *)&_C);

    //mat_mult(&_A, &_B, &_C);
}

// At = A^T
void EKF::device_mat_trans(
        const float A[STATE_DIM][STATE_DIM],
        float At[STATE_DIM][STATE_DIM])

{
    static __attribute__((aligned(4))) arm_matrix_instance_f32 _A = { 
        STATE_DIM, STATE_DIM, (float *)A
    };

    arm_matrix_instance_f32 _At = {};
    _At.numRows = STATE_DIM;
    _At.numCols = STATE_DIM;
    _At.pData = (float*)At;

    arm_mat_trans_f32(
            (arm_matrix_instance_f32 *)&_A,
            (arm_matrix_instance_f32 *)&_At);
}

void EKF::device_predict(const float F[STATE_DIM][STATE_DIM],
        float P[STATE_DIM][STATE_DIM])
{
    static float FP[STATE_DIM * STATE_DIM];

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _F = { 
        STATE_DIM, STATE_DIM, (float *)F
    };

    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { 
        STATE_DIM, STATE_DIM, FP
    };

    static float tmpNN2d[STATE_DIM * STATE_DIM];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { 
        STATE_DIM, STATE_DIM, tmpNN2d
    };

    arm_matrix_instance_f32 _P = {};
    _P.numRows = STATE_DIM;
    _P.numCols = STATE_DIM;
    _P.pData = (float*)P;

    mat_mult(&_F, &_P, &tmpNN1m); // F P
    mat_trans(&_F, &tmpNN2m); // F'

    mat_mult(&tmpNN1m, &tmpNN2m, &_P); // F P F'
}

void EKF::device_update_with_scalar(
        const float P[STATE_DIM][STATE_DIM],
        const float * h,
        const float error,
        const float R,
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
    }

    mat_mult(&Gm, &Hm, &tmpNN1m); // GH
    for (int i=0; i<STATE_DIM; i++) { 
        tmpNN1d[STATE_DIM*i+i] -= 1; 
    } // GH - I
    mat_trans(&tmpNN1m, &tmpNN2m); // (GH - I)'
    mat_mult(&tmpNN1m, &_p_m, &tmpNN3m); // (GH - I)*P
    mat_mult(&tmpNN3m, &tmpNN2m, &_p_m); // (GH - I)*P*(GH - I)'
}
