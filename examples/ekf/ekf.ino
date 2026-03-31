/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <ArduinoEigenDense.h>

#include <arm_math.h>

static constexpr float MAX_COVARIANCE = 100;
static constexpr float MIN_COVARIANCE = 1e-6;

static const size_t STATE_DIM = 3;

// ---------------------------------------------------------------------------

typedef arm_matrix_instance_f32 matrix_t;

static void device_mat_trans(const matrix_t * pSrc, matrix_t * pDst)
{
    arm_mat_trans_f32((arm_matrix_instance_f32 *)pSrc,
            (arm_matrix_instance_f32 *)pDst);
}

static void device_mat_mult(
        const matrix_t * pSrcA, const matrix_t * pSrcB,
        matrix_t * pDst) 
{
    arm_mat_mult_f32((arm_matrix_instance_f32 *)pSrcA, (arm_matrix_instance_f32 *)pSrcB,
            (arm_matrix_instance_f32 *)pDst);
}

static void pset(
        float  P[STATE_DIM][STATE_DIM],
        const size_t i,
        const size_t j,
        const float pval)
{
    if (isnan(pval) || pval > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
    } else if ( i==j && pval < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
    } else {
        P[i][j] = P[j][i] = pval;
    }
}

static void run_old(
            float Pvals[STATE_DIM][STATE_DIM],
            float xvals[STATE_DIM],
            const float hvals[STATE_DIM],
            const float R,
            const float error)
{
    matrix_t _p_m;

    _p_m.numRows = STATE_DIM;
    _p_m.numCols = STATE_DIM;
    _p_m.pData = (float*)Pvals;

    matrix_t Hm = {1, STATE_DIM, (float *)hvals};

    // The Kalman gain as a column vector
    static float G[STATE_DIM];
    static matrix_t Gm = {STATE_DIM, 1, (float *)G};

    // Temporary matrices for the covariance updates
    static float tmpNN1d[STATE_DIM * STATE_DIM];
    static matrix_t tmpNN1m = {
        STATE_DIM, STATE_DIM, tmpNN1d
    };

    static float tmpNN2d[STATE_DIM * STATE_DIM];
    static matrix_t tmpNN2m = {
        STATE_DIM, STATE_DIM, tmpNN2d
    };

    static float tmpNN3d[STATE_DIM * STATE_DIM];
    static matrix_t tmpNN3m = {
        STATE_DIM, STATE_DIM, tmpNN3d
    };


    static float HTd[STATE_DIM * 1];
    static matrix_t HTm = {STATE_DIM, 1, HTd};

    static float PHTd[STATE_DIM * 1];
    static matrix_t PHTm = {STATE_DIM, 1, PHTd};

    device_mat_trans(&Hm, &HTm);
    device_mat_mult(&_p_m, &HTm, &PHTm); // PH'
    float HPHR = R; // HPH' + R
    for (size_t i=0; i<STATE_DIM; i++) { 
        // Add the element of HPH' to the above
        // this obviously only works if the update is scalar (as in this function)
        HPHR += Hm.pData[i]*PHTd[i]; 
    }

    // Calculate the Kalman gain and perform the state update
    for (size_t i=0; i<STATE_DIM; i++) {
        G[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
        xvals[i] = xvals[i] + G[i] * error; // state update
    }

    device_mat_mult(&Gm, &Hm, &tmpNN1m); // GH
    for (size_t i=0; i<STATE_DIM; i++) { 
        tmpNN1d[STATE_DIM*i+i] -= 1; 
    } // GH - I
    device_mat_trans(&tmpNN1m, &tmpNN2m); // (GH - I)'
    device_mat_mult(&tmpNN1m, &_p_m, &tmpNN3m); // (GH - I)*P
    device_mat_mult(&tmpNN3m, &tmpNN2m, &_p_m); // (GH - I)*P*(GH - I)'

    // add the measurement variance and ensure boundedness and symmetry
    for (size_t i=0; i<STATE_DIM; i++) {

        for (size_t j=i; j<STATE_DIM; j++) {

            float v = G[i] * R * G[j];

            // add measurement noise
            pset(Pvals, i, j, 0.5 * Pvals[i][j] + 0.5 * Pvals[j][i] + v); 
        }
    }
}

// ---------------------------------------------------------------------------

static void run_new(
        float Pvals[STATE_DIM][STATE_DIM],
        float xvals[STATE_DIM],
        const float hvals[STATE_DIM],
        const float R,
        const float error)
{
    (void)Pvals;
    (void)xvals;
    (void)hvals;
    (void)R;
    (void)error;
}

// ---------------------------------------------------------------------------

static void report(
        const float P[STATE_DIM][STATE_DIM],
        const float x[STATE_DIM])
{
    for (size_t i=0; i<STATE_DIM; ++i) {
        for (size_t j=0; j<STATE_DIM; ++j) {
            printf("%+f ", P[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}


void setup()
{
}

void loop()
{
    float P[STATE_DIM][STATE_DIM] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };

    const float h[STATE_DIM] = {10, 11, 12};

    float x[STATE_DIM] = {13, 14, 15};

    const auto stdMeasNoise = 1.5f;

    const auto error = -2.3;

    const auto R = stdMeasNoise*stdMeasNoise;

    run_old(P, x, h, R, error);

    report(P, x);

    printf("\n");

    run_new(P, x, h, R, error);

    printf("\n");

    delay(1000);
}
