/**
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

#include <firmware/estimator/quaternion.hpp>
#include <firmware/estimator/three_axis_subsampler.hpp>
#include <firmware/flow_filter.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/zranger/filter.hpp>
#include <num.hpp>

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

            // State vector
            __attribute__((aligned(4))) float x[STATE_DIM];

            // Covariance matrix
            __attribute__((aligned(4))) float P[STATE_DIM][STATE_DIM];


            EkfCore() 
            {
                for (int i=0; i< STATE_DIM; i++) {

                    x[i] = 0;

                    for (int j=0; j < STATE_DIM; j++) {
                        P[i][j] = 0; 
                    }
                }
            }

            void enforceSymmetry(const float minval, const float maxval)
            {
                for (int i=0; i<STATE_DIM; i++) {

                    for (int j=i; j<STATE_DIM; j++) {

                        P[i][j] = P[j][i] =
                            get_pval(i, j, 0.5*P[i][j] + 0.5*P[j][i],
                                    minval, maxval);
                    }
                }
            }

            static auto get_pval(const int i, const int j,
                    const float pval, const float minval,
                    const float maxval) -> float
            {
                return
                    isnan(pval) || pval > maxval ? maxval :
                    i==j && pval < minval ? minval :
                    pval;
            }


            void addCovarianceNoise(const float * noise)
            {
                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    P[k][k] += noise[k] * noise[k];
                }
            }

            // C = A * B
            static void dot(
                    const float A[STATE_DIM][STATE_DIM],
                    const float B[STATE_DIM][STATE_DIM],
                    float C[STATE_DIM][STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        C[i][j] = 0;
                        for (int k=0; k<STATE_DIM; ++k) {
                            C[i][j] += A[i][k] * B[k][j];
                        }
                    }
                }
            }

            // y = A * x
            static void dot(
                    const float A[STATE_DIM][STATE_DIM],
                    const float x[STATE_DIM],
                    float y[STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; i++) {
                    y[i] = 0; 
                    for (int j=0; j<STATE_DIM; j++) {
                        y[i] += A[i][j] * x[j];
                    }
                }
            }

            // A = x * y
            static void outer(
                    const float x[STATE_DIM],
                    const float y[STATE_DIM],
                    float C[STATE_DIM][STATE_DIM])
            {
                for (size_t i=0; i<STATE_DIM; i++) {
                    for (size_t j=0; j<STATE_DIM; j++) {
                        C[i][j] = x[i] * y[j];
                    }
                }
            }

            // At = A^T
            static void trans(
                    const float A[STATE_DIM][STATE_DIM],
                    float At[STATE_DIM][STATE_DIM])
            {
                for (int i=0; i<STATE_DIM; ++i) {
                    for (int j=0; j<STATE_DIM; ++j) {
                        At[i][j] = A[j][i];
                    }
                }
            }
    };
}
