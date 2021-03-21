/*
   Support for USFSMAX IMU

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include <USFSMAX_Basic.h>

#include "sensor.hpp"

namespace hf {

    // Singleton class
    class _USFSMAX {

        friend class UsfsQuat;
        friend class UsfsGyro;

        private:

        // Tunable USFSMAX parameters

        // Magnetic constants for Lexington, VA, USA
        // For your location, use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
        static constexpr float MAG_V           = 45.821;  // vertical intensity (uT)
        static constexpr float MAG_H           = 21.521;  // horizontal intensity (uT)
        static constexpr float MAG_DECLINATION = -9.1145; // angle, degrees

        // Serial update period (ms)
        static const uint32_t UPDATE_PERIOD  = 100;

        // I2C Clock Speed
        static const uint32_t I2C_CLOCK = 1000000;    // 1MHz

        // Output Data Rates (ODRs)
        static const USFSMAX::AccelGyroODR_t ACCEL_ODR = USFSMAX::ACCEL_GYRO_ODR_834;
        static const USFSMAX::AccelGyroODR_t GYRO_ODR  = USFSMAX::ACCEL_GYRO_ODR_834;
        static const USFSMAX::MagODR_t       MAG_ODR   = USFSMAX::MAG_ODR_100;
        static const USFSMAX::BaroODR_t      BARO_ODR  = USFSMAX::BARO_ODR_50;
        static const USFSMAX::QuatDiv_t      QUAT_DIV  = USFSMAX::QUAT_DIV_8;

        // LSM6DSM filter settings
        static const USFSMAX::LSM6DSMGyroLPF_t   LSM6DSM_GYRO_LPF    = USFSMAX::LSM6DSM_GYRO_LPF_167;
        static const USFSMAX::LSM6DSMAccLpfODR_t LSM6DSM_ACC_LPF_ODR = USFSMAX::LSM6DSM_ACC_LPF_ODR_DIV400;

        // LIS2MDL filter setting
        static const USFSMAX::LIS2MDLMagLpfODR_t LIS2MDL_MAG_LPF_ODR = USFSMAX::LIS2MDL_MAG_LPF_ODR_4;

        // LPS22HB baro filter setting
        static const USFSMAX::LPS22HBBaroLpfODR_t LPS22HB_BARO_LPF = USFSMAX::LPS22HB_BARO_LPF_ODR_20;

        // IMU scaling
        static const USFSMAX::AccScale_t  ACC_SCALE  = USFSMAX::ACC_SCALE_16;
        static const USFSMAX::GyroScale_t GYRO_SCALE = USFSMAX::GYRO_SCALE_2000;

        bool _begun = false;

        protected:

        USFSMAX_Basic usfsmax =
            USFSMAX_Basic(
                    ACCEL_ODR,
                    GYRO_ODR,
                    MAG_ODR,
                    BARO_ODR,
                    QUAT_DIV,
                    LSM6DSM_GYRO_LPF,
                    LSM6DSM_ACC_LPF_ODR,
                    ACC_SCALE,
                    GYRO_SCALE,
                    LIS2MDL_MAG_LPF_ODR,
                    LPS22HB_BARO_LPF,
                    MAG_V,
                    MAG_H,
                    MAG_DECLINATION);

        void begin(void)
        {
            if (_begun) return;

            _begun = true;
        }

    }; // class _USFS

    static _USFSMAX  _usfsmax;

    class UsfsQuat : public Sensor {

        private:

            static void computeEulerAngles(float qw, float qx, float qy, float qz,
                    float & ex, float & ey, float & ez)
            {
                ex = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
                ey = asin(2.0f*(qx*qz-qw*qy));
                ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
            }

        protected:

            virtual void begin(void) override 
            {
                _usfsmax.begin();
            }

            virtual void modifyState(State * state, float time) override
            {
                (void)time;

                float qw = 0;
                float qx = 0;
                float qy = 0;
                float qz = 0;

                //_usfs.sentral.readQuaternion(qw, qx, qy, qz);

                computeEulerAngles(qw, qx, qy, qz,
                        state->x[State::STATE_PHI],
                        state->x[State::STATE_THETA],
                        state->x[State::STATE_PSI]);

                // Adjust rotation so that nose-up is positive
                state->x[State::STATE_THETA] = -state->x[State::STATE_THETA];

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (state->x[State::STATE_PSI] < 0) {
                    state->x[State::STATE_PSI] += 2*M_PI;
                }
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return false;
            }

    }; // class UsfsQuat


    class UsfsGyro : public Sensor {

        protected:

            virtual void begin(void) override 
            {
                _usfsmax.begin();
            }

            virtual void modifyState(State * state, float time) override
            {
                (void)time;

                float gx = 0;
                float gy = 0;
                float gz = 0;

                // Returns degrees / sec
                // _usfs.sentral.readGyrometer(gx, gy, gz);

                // Convert degrees / sec to radians / sec
                state->x[State::STATE_DPHI] = radians(gx);
                state->x[State::STATE_DTHETA] = radians(gy);
                state->x[State::STATE_DPSI] = radians(gz);
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return false;
            }

    }; // class UsfsGyro

} // namespace hf
