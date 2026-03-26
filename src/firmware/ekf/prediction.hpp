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

        private:

            static constexpr float G = 9.81;

            // Small number epsilon, to prevent dividing by zero
            static constexpr float EPSILON = 1e-6f;

            // the reversion of pitch and roll to zero
            static constexpr float ROLLPITCH_ZERO_REVERSION = 0.001;

            // Initial variances, uncertain of position, but know we're
            // stationary and roughly flat
            static constexpr float STDEV_INITIAL_POSITION_XY = 100;
            static constexpr float STDEV_INITIAL_POSITION_Z = 1;
            static constexpr float STDEV_INITIAL_VELOCITY = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_ROLLPITCH = 0.01;
            static constexpr float STDEV_INITIAL_ATTITUDE_YAW = 0.01;

            // The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
            static constexpr float MAX_COVARIANCE = 100;
            static constexpr float MIN_COVARIANCE = 1e-6;

            static constexpr float MIN_VELOCITY_MPS = 1e-4;
            static constexpr float MAX_VELOCITY_MPS = 10;

        public:

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

            // State vector
            __attribute__((aligned(4))) Eigen::VectorXd x =
                Eigen::VectorXd(STATE_DIM);

            // The vehicle's attitude as a quaternion (w,x,y,z) We store as a quaternion
            // to allow easy normalization (in comparison to a rotation matrix),
            // while also being robust against singularities (in comparison to euler angles)
            Eigen::VectorXd q = Eigen::VectorXd(4);

            // Covariance matrix
            Eigen::MatrixXd _P = Eigen::MatrixXd(STATE_DIM, STATE_DIM);

            Prediction()
            {
                // Reset the state
                x = Eigen::VectorXd(STATE_DIM);

                // Reset the attitude quaternion
                q << 1, 0, 0, 0;

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

            Prediction(

                    const Eigen::VectorXd & x,
                    const Eigen::VectorXd & q,
                    const Eigen::MatrixXd & P)
                : x(x), q(q), _P(P) {}

            void run(
                    const Vec3 & accel,
                    const Vec3 & gyro,
                    const float dt,
                    const bool isFlying,
                    const Eigen::MatrixXd & R)
            {
           }

            static bool isVelInBounds(const float vel)
            {
                return fabs(vel) < MAX_VELOCITY_MPS;
            }

        private:

            static auto makeJacobian(
                    const Eigen::VectorXd & x,
                    const Eigen::MatrixXd & R,
                    const Vec3 & gyro,
                    const Vec3 & accel,
                    const float dt) -> Eigen::MatrixXd
            {
                const auto d0 = gyro.x*dt/2;
                const auto d1 = gyro.y*dt/2;
                const auto d2 = gyro.z*dt/2;

                const auto vx = x(1);
                const auto vy = x(2);
                const auto vz = x(3);

                Eigen::MatrixXd F(STATE_DIM, STATE_DIM);

                // position
                F(0, 0) = 1;

                // position from body-frame velocity
                F(0,1) = R(2,0)*dt;

                F(0,2) = R(2,1)*dt;

                F(0,3) = R(2,2)*dt;

                // position from attitude error
                F(0,4) = (vy*R(2,2) - vz*R(2,1))*dt;

                F(0,5) = (-vx*R(2,2) + vz*R(2,0))*dt;

                F(0,6) = (vx*R(2,1) - vy*R(2,0))*dt;

                // body-frame velocity from body-frame velocity
                F(1,1) = 1; //drag negligible
                F(2,1) =-gyro.z*dt;
                F(3,1) = gyro.y*dt;

                F(1,2) = gyro.z*dt;
                F(2,2) = 1; //drag negligible
                F(3,2) =-gyro.x*dt;

                F(1,3) =-gyro.y*dt;
                F(2,3) = gyro.x*dt;
                F(3,3) = 1; //drag negligible

                // body-frame velocity from attitude error
                F(1,4) =  0;
                F(2,4) = -G*R(2,2)*dt;
                F(3,4) =  G*R(2,1)*dt;

                F(1,5) =  G*R(2,2)*dt;
                F(2,5) =  0;
                F(3,5) = -G*R(2,0)*dt;

                F(1,6) = -G*R(2,1)*dt;
                F(2,6) =  G*R(2,0)*dt;
                F(3,6) =  0;

                F(4,4) =  1 - d1*d1/2 - d2*d2/2;
                F(4,5) =  d2 + d0*d1/2;
                F(4,6) = -d1 + d0*d2/2;

                F(5,4) = -d2 + d0*d1/2;
                F(5,5) =  1 - d0*d0/2 - d2*d2/2;
                F(5,6) =  d0 + d1*d2/2;

                F(6,4) =  d1 + d0*d2/2;
                F(6,5) = -d0 + d1*d2/2;
                F(6,6) = 1 - d0*d0/2 - d1*d1/2;

                return F;
            }

            static auto addCovarianceNoise(const Eigen::MatrixXd & P,
                    const float * noise) -> Eigen::MatrixXd
            {
                auto Pcov = P;

                for (uint8_t k=0; k<STATE_DIM; ++k) {
                    Pcov(k,k) += noise[k] * noise[k];
                }

                return Pcov;
            }

             static bool isVelPositive(const float vel)
            {
                return fabs(vel) > MIN_VELOCITY_MPS;
            }
    };
}
