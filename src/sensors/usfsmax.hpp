/*
   Support for USFSMAX IMU

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include <USFSMAX_Basic.h>
#include <RFT_debugger.hpp>
#include <RFT_sensor.hpp>

namespace hf {

    // Singleton class
    class _USFSMAX {

        friend class UsfsMaxQuaternion;
        friend class UsfsMaxGyrometer;

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

        static void error(uint8_t status)
        {
            while (true) {
                rft::Debugger::printf("Got error %d\n", status);
                delay(500);
            }
        }

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

            // Initialize I^2C bus, setting I2C clock speed to 100kHz
            Wire.begin();
            delay(100);
            Wire.setClock(100000); 
            delay(1000);

            uint8_t status = usfsmax.begin(); // Start USFSMAX

            if (status) {
                error(status);
            }

            Wire.setClock(I2C_CLOCK);// Set the I2C clock to high speed for run-mode data collection
            delay(100);

            _begun = true;
        }

        bool gyroReady(void)
        {
            switch (usfsmax.dataReady()) {
                case USFSMAX::DATA_READY_GYRO_ACC:
                    return true;
                case USFSMAX::DATA_READY_GYRO_ACC_MAG_BARO:
                    return true;
            }

            return false;
        }

        void readGyro(float gyro[3])
        {
            float acc[3] = {};

            usfsmax.readGyroAcc(gyro, acc);
        }

        bool quaternionReady(void)
        {
            return usfsmax.quaternionReady();
        }

        void readQuaternion(float quat[4])
        {
            usfsmax.readQuat(quat);
        }

    }; // class _USFS

    static _USFSMAX  _usfsmax;

    class UsfsMaxQuaternion : public rft::Sensor {

        protected:

            virtual void begin(void) override 
            {
                _usfsmax.begin();
            }

            virtual void modifyState(rft::State * state, float time) override
            {
                (void)time;

                float q[4] = {};
                _usfsmax.readQuaternion(q);

                float qw = q[0];
                float qx = q[1];
                float qy = q[2];
                float qz = q[3];

                float ex = -atan2(2.0f*(qw*qx+qy*qz),qw*qw-qx*qx-qy*qy+qz*qz);
                float ey = -asin(2.0f*(qx*qz-qw*qy));
                float ez = -atan2(2.0f*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz);

                State * hfstate = (State *)state;

                hfstate->x[State::PHI] = ex;
                hfstate->x[State::THETA] = ey;
                hfstate->x[State::PSI] = ez;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return _usfsmax.quaternionReady();
            }

    }; // class UsfsQuat

    class UsfsMaxGyrometer : public rft::Sensor {

        protected:

            virtual void begin(void) override 
            {
                _usfsmax.begin();
            }

            virtual void modifyState(rft::State * state, float time) override
            {
                (void)time;

                float gyro[3] = {};
                _usfsmax.readGyro(gyro);

                State * hfstate = (State *)state;

                // Convert degrees / sec to radians / sec
                hfstate->x[State::DPHI] = -radians(gyro[0]);
                hfstate->x[State::DTHETA] = radians(gyro[1]);
                hfstate->x[State::DPSI] = radians(gyro[2]);
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return _usfsmax.gyroReady();
            }

    }; // class UsfsGyro

} // namespace hf
