/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2026 Simon D. Levy
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

#include <array>

namespace hf {

    class EkfCore { 

        public:

            // Indexes to acceless the vehicle's state, stored as a column vector
            enum
            {
                STATE_Z,
                STATE_VX,
                STATE_VY,
                STATE_VZ,
                STATE_D0,
                STATE_D1,
                STATE_D2,
                STATE_DIM
            };

            typedef std::array<float, STATE_DIM*STATE_DIM> matrix;

            typedef std::array<float, STATE_DIM> vector;

            // State vector
            vector x;

            EkfCore()
            {
                x = vector();
                P = matrix();
            }

            // P_k = F_{k-1} P_{k-1} F^T_{k-1} --------------------
            void predict(const matrix & F)
            {
                const auto FP = dot(F, P);

                const auto Ft = trans(F);

                P = dot(FP, Ft);
            }

            void updateWithScalar(
                    const vector & h,
                    const float error,
                    const float stdMeasNoise,
                    const float minCovariance,
                    const float maxCovariance)
            {
                const auto R = stdMeasNoise*stdMeasNoise;

                const auto PHt = dot(P, h); // PH'

                float HPHR = R; // HPH' + R
                for (size_t i=0; i<STATE_DIM; i++) { 
                    HPHR += h[i] * PHt[i]; 
                }

                vector G;
                for (size_t i=0; i<STATE_DIM; i++) {
                    G[i] = PHt[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
                }

                auto GH = outer(G, h);

                // GH - I
                for (size_t i=0; i<STATE_DIM; i++) { 
                    GH[i*STATE_DIM+i] -= 1; 
                }

                // (GH - I)'
                const auto GH_I = trans(GH);

                // (GH - I)*P
                const auto GH_I_P = dot(GH, P); 

                // (GH - I)*P*(GH - I)'
                P = dot(GH_I_P, GH_I);

                // State update
                for (int i=0; i<STATE_DIM; i++) {
                    x[i] += G[i] * error; // state update
                }

                // Add the measurement variance and ensure boundedness and symmetry
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        const auto v = G[i] * R * G[j];

                        // add measurement noise
                        P[i*STATE_DIM+j] = P[j*STATE_DIM+i] =
                            get_pval(i, j, 0.5*P[i*STATE_DIM+j] + 0.5*P[j*STATE_DIM+i] + v,
                                    minCovariance, maxCovariance); 
                    }
                }

            }

            void enforceSymmetry(const float minval, const float maxval)
            {
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        P[i*STATE_DIM+j] = P[j*STATE_DIM+i] =
                            get_pval(i, j, 0.5*P[i*STATE_DIM+j] + 0.5*P[j*STATE_DIM+i],
                                    minval, maxval);
                    }
                }
            }

            void addCovarianceNoise(const float * noise)
            {
                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    P[k*STATE_DIM+k] += noise[k] * noise[k];
                }
            }

        private:

            // Covariance matrix
            matrix P;

            static auto get_pval(const int i, const int j,
                    const float pval, const float minval,
                    const float maxval) -> float
            {
                return
                    isnan(pval) || pval > maxval ? maxval :
                    i==j && pval < minval ? minval :
                    pval;
            }

            // C = x * y
            static auto outer(const vector & x, const vector & y) -> matrix
            {
                auto C = matrix();

                for (size_t i=0; i<STATE_DIM; i++) {
                    for (size_t j=0; j<STATE_DIM; j++) {
                        C[i*STATE_DIM+j] = x[i] * y[j];
                    }
                }

                return C;
            }

            // At = A^T
            static auto trans(const matrix & A) -> matrix
            {
                auto At = matrix();

                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        At[i*STATE_DIM+j] = A[j*STATE_DIM+i];
                    }
                }

                return At;
            }

            // C = A * B
            static auto dot(const matrix & A, const matrix & B) -> matrix
            {
                auto C = matrix();

                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        C[i*STATE_DIM+j] = 0;
                        for (int k=0; k<STATE_DIM; ++k) {
                            C[i*STATE_DIM+j] += A[i*STATE_DIM+k] * B[k*STATE_DIM+j];
                        }
                    }
                }

                return C;
            }

            // y = A * x
            static auto dot(const matrix & A, const vector & x) -> vector
            {
                vector y = vector();

                for (int i=0; i<STATE_DIM; i++) {
                    y[i] = 0; 
                    for (int j=0; j<STATE_DIM; j++) {
                        y[i] += A[i*STATE_DIM+j] * x[j];
                    }
                }

                return y;
            }

    };
}
