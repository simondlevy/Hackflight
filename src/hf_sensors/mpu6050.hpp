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
