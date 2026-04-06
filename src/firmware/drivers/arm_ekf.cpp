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

static void arm_mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{
    arm_mat_trans_f32((arm_matrix_instance_f32 *)pSrc,
            (arm_matrix_instance_f32 *)pDst);
}

static void arm_mat_mult(
        const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB,
        arm_matrix_instance_f32 * pDst) 
{
    arm_mat_mult_f32((arm_matrix_instance_f32 *)pSrcA, (arm_matrix_instance_f32 *)pSrcB,
            (arm_matrix_instance_f32 *)pDst);
}

//////////////////////////////////////////////////////////////////////////////

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

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _C = { 
        STATE_DIM, STATE_DIM, (float *)C
    };

    arm_mat_mult(&_A, &_B, &_C);
}

void EKF::device_mat_trans(
        const float A[STATE_DIM][STATE_DIM],
        float At[STATE_DIM][STATE_DIM])
{
    static __attribute__((aligned(4))) arm_matrix_instance_f32 _A = { 
        STATE_DIM, STATE_DIM, (float *)A
    };

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _At = { 
        STATE_DIM, STATE_DIM, (float *)At
    };

    arm_mat_trans(&_A, &_At);
}



//////////////////////////////////////////////////////////////////////////////

void EKF::device_predict(const float F[STATE_DIM][STATE_DIM],
        float P[STATE_DIM][STATE_DIM])
{
    // ------------------------------------------------------------------

    float FP[STATE_DIM][STATE_DIM] = {};

    device_mat_mult(F, P, FP);

    // ------------------------------------------------------------------

    float Ft[STATE_DIM][STATE_DIM] = {};

    device_mat_trans(F, Ft); // F'

    // ------------------------------------------------------------------

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _P = { 
        STATE_DIM, STATE_DIM, (float *)P
    };

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _Ft = { 
        STATE_DIM, STATE_DIM, (float *)Ft
    };

    static __attribute__((aligned(4))) arm_matrix_instance_f32 _FP = { 
        STATE_DIM, STATE_DIM, (float*)FP
    };

    arm_mat_mult(&_FP, &_Ft, &_P); // F P F'
}

void EKF::device_update_with_scalar(
        const float P[STATE_DIM][STATE_DIM],
        const float h[STATE_DIM],
        const float error,
        const float R,
        float G[STATE_DIM])
{
    arm_matrix_instance_f32 Hm = {1, STATE_DIM, (float *)h};

    static float Ht[STATE_DIM * 1];
    static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, Ht};

    static float PHt[STATE_DIM * 1];
    static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHt};

    arm_matrix_instance_f32 Pm = {};
    Pm.numRows = STATE_DIM;
    Pm.numCols = STATE_DIM;
    Pm.pData = (float*)P;

    arm_mat_trans(&Hm, &HTm);
    arm_mat_mult(&Pm, &HTm, &PHTm); // PH'

    float HPHR = R; // HPH' + R
    for (int i=0; i<STATE_DIM; i++) { 
        // Add the element of HPH' to the above
        // this obviously only works if the update is scalar (as in this function)
        HPHR += Hm.pData[i]*PHt[i]; 
    }

    // Calculate the Kalman gain and perform the state update
    for (int i=0; i<STATE_DIM; i++) {
        G[i] = PHt[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    }

    // Temporary matrices for the covariance updates
    static float GH[STATE_DIM][STATE_DIM];
    static arm_matrix_instance_f32 GHm = { STATE_DIM, STATE_DIM, (float *)GH };
    static float tmpNN2d[STATE_DIM * STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = {
        STATE_DIM, STATE_DIM, tmpNN2d
    };
    static float tmpNN3d[STATE_DIM * STATE_DIM];
    static arm_matrix_instance_f32 tmpNN3m = {
        STATE_DIM, STATE_DIM, tmpNN3d
    };

    // The Kalman gain as a column vector
    static arm_matrix_instance_f32 Gm = {STATE_DIM, 1, (float *)G};

    // GH
    arm_mat_mult(&Gm, &Hm, &GHm);

     // GH - I
    for (int i=0; i<STATE_DIM; i++) { 
        GH[i][i] -= 1; 
    }

    // (GH - I)'
    arm_mat_trans(&GHm, &tmpNN2m); 

    // (GH - I)*P
    arm_mat_mult(&GHm, &Pm, &tmpNN3m); 

    // (GH - I)*P*(GH - I)'
    arm_mat_mult(&tmpNN3m, &tmpNN2m, &Pm); 
}
