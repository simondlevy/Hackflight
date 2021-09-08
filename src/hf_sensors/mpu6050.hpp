/*
   Support for Invense MPU 6050 IMU

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include <RFT_sensor.hpp>
#include <RFT_filters.hpp>

// MPU I^2C
static const int MPU_ADDR = 0x68;
static const uint8_t POWER_ON     = 0x6B;
static const uint8_t ACCEL_XOUT_H = 0x3B;
static const uint8_t GYRO_XOUT_H  = 0x43;

namespace hf {

    class MPU6050 : public rft::Sensor {

        friend class Hackflight;

        private:

        // From https://github.com/kriswiner/MPU9250/blob/master/MPU9250BasicAHRS.ino

        // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
        static constexpr float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
        static constexpr float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

        // There is a tradeoff in the beta parameter between accuracy and
        // response speed.  In the original Madgwick study, beta of 0.041
        // (corresponding to GyroMeasError of 2.7 degrees/s) was found to give
        // optimal accuracy.  However, with this value, the LSM9SD0 response
        // time is about 10 seconds to a stable initial quaternion.  Subsequent
        // changes also require a longish lag time to a stable output, not fast
        // enough for a quadcopter or robot car!  By increasing beta
        // (GyroMeasError) by about a factor of fifteen, the response time
        // constant is reduced to ~2 sec I haven't noticed any reduction in
        // solution accuracy. This is essentially the I coefficient in a PID
        // control sense; the bigger the feedback coefficient, the faster the
        // solution converges, usually at the expense of accuracy.  In any
        // case, this is the free parameter in the Madgwick filtering and
        // fusion scheme.
        static constexpr float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   
        static constexpr float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // usually set to a small or zero value

        rft::MadgwickQuaternionFilter6DOF madgwick = rft::MadgwickQuaternionFilter6DOF(beta, zeta);


        // Reads the gyro or the accelerometer
        static void readImu(const uint8_t reg, int16_t sensor[3])
        {
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(reg);
            Wire.endTransmission(false);
            Wire.requestFrom(MPU_ADDR, 14, 1);

            sensor[0] = Wire.read() << 8 | Wire.read();
            sensor[1] = Wire.read() << 8 | Wire.read();
            sensor[2] = Wire.read() << 8 | Wire.read();
        }

        protected:

        void begin(void)
        {
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(POWER_ON);
            Wire.write(0);
            Wire.endTransmission(true);
        }

        virtual void modifyState(rft::State * state, float time)
        {
            State * hfstate = (State *)state;

            (void)time;

            int16_t accel[3] = {};
            readImu(ACCEL_XOUT_H, accel);

            int16_t gyro[3] = {};
            readImu(GYRO_XOUT_H, gyro);

            Serial.println(gyro[0]);

        } // modifyState

    };  // class MPU6050

} // namespace hf
