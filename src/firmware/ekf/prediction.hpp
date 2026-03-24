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

#include <firmware/datatypes.hpp>
#include <firmware/ekf/imu_subsampler.hpp>
#include <firmware/timer.hpp>
#include <num.hpp>

// We want to use F for the Jacobian and _P for the covariance matrix, but
// Arduino pre-defines them
#ifdef F
#undef F
#undef _P
#endif

namespace hf {

    class Prediction { 

        friend class EKF;

        private:

            static constexpr float G = 9.81;

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float STDEV_INITIAL_POSITION_XY = 100;
            static constexpr float STDEV_INITIAL_POSITION_Z = 1;
            static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_ROLLPITCH = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

            // Indexes to access the vehicle's state, stored as a column vector
            enum {
                STATE_Z,
                STATE_VX,
                STATE_VY,
                STATE_VZ,
                STATE_D0,
                STATE_D1,
                STATE_D2,
                STATE_DIM
            };

            Prediction()
            {
                _accelSubSampler = ImuSubSampler(G);
                _gyroSubSampler = ImuSubSampler(Num::DEG2RAD);

                // Reset the state
                _x = Eigen::VectorXd(STATE_DIM);

                // Reset the attitude quaternion
                _q << 1, 0, 0, 0;

                // Reset the covariance matrix and add the initial process
                // noise
                _P = Eigen::MatrixXd(STATE_DIM, STATE_DIM);
                const float pinit[STATE_DIM] = {
                    STDEV_INITIAL_POSITION_Z,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_VELOCITY,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_ROLLPITCH,
                    STDEV_INITIAL_ATTITUDE_YAW
                };
                _P = addCovarianceNoise(_P, pinit);
            }

            Prediction& operator=(const Prediction& other) = default;

            /*
            Prediction(

                    const Eigen::VectorXd & x,
                    const Eigen::MatrixXd & P,
                    const ImuSubSampler & accelSubSampler,
                    const ImuSubSampler & gyroSubSampler,
                    const Eigen::VectorXd & q)
                :
                    _x(x),
                    _P(P),
                    _accelSubSampler(accelSubSampler),
                    _gyroSubSampler(gyroSubSampler),
                    _q(q) { }
                    */

            // State vector
            __attribute__((aligned(4))) Eigen::VectorXd _x =
                Eigen::VectorXd(STATE_DIM);

            // Covariance matrix
            Eigen::MatrixXd _P = Eigen::MatrixXd(STATE_DIM, STATE_DIM);

            ImuSubSampler _accelSubSampler;
            ImuSubSampler _gyroSubSampler;

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Eigen::VectorXd _q = Eigen::VectorXd(4);

            static auto addCovarianceNoise(const Eigen::MatrixXd & P,
                    const float * noise) -> Eigen::MatrixXd
            {
                auto Pcov = P;

                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    Pcov(k,k) += noise[k] * noise[k];
                }

                return Pcov;
            }
    };
}
