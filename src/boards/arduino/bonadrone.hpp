/*
   bonadrone.hpp : Implementation of Hackflight Board routines for Bonadrone Flight Controller

   Copyright (c) 2018 Juan Gallostra Acin, Simon D. Levy, Pep Martí Saumell

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

#include <Wire.h>

#include <LSM6DSM.h> 

#include "filters.hpp"
#include "hackflight.hpp"
#include "boards/softquat.hpp"
#include "boards/arduino/arduino.hpp"
#include "motors/brushed.hpp"
#include "motors/standard.hpp"
#include "motors/multishot.hpp"

namespace hf {

    class BonadroneBoard : public ArduinoBoard, public SoftwareQuaternionBoard {

        private:

            // LSM6DSM data-ready interrupt pin
            const uint8_t LSM6DSM_INTERRUPT_PIN = 2;

            // Paramters to experiment with ------------------------------------------------------------------------

            // LSM6DSM full-scale settings
            static const LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_8G;
            static const LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_2000DPS;
            static const LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_1660Hz;
            static const LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_1660Hz;

            // Biases computed by Simon using Juan & Pep's LSM6DSM/Examples/Calibrate sketch
            float ACCEL_BIAS[3] = {-0.020306,0.008926,0.029526};
            float GYRO_BIAS[3]  = {0.301350,-0.818594,-0.701652};


            // Instance variables -----------------------------------------------------------------------------------

            LSM6DSM _lsm6dsm = LSM6DSM(Ascale, Gscale, AODR, GODR, ACCEL_BIAS, GYRO_BIAS);

            // Helpers

            void i2cerror(const char * devicename)
            {
                while (true) {
                    Serial.print("Unable to start: ");
                    Serial.println(devicename);
                }
            }

        protected:


            const uint8_t MOTOR_PIN_1 = 3;
            const uint8_t MOTOR_PIN_2 = 4;
            const uint8_t MOTOR_PIN_3 = 5;
            const uint8_t MOTOR_PIN_4 = 6;

            virtual void writeMotor(uint8_t index, float value) = 0;

            virtual bool  getQuaternion(float & qw, float & qx, float & qy, float & qz) override 
            {
                return SoftwareQuaternionBoard::getQuaternion(qw, qx, qy, qz, getTime());
            }

            virtual bool  getGyrometer(float & gx, float & gy, float & gz) override
            {
                return SoftwareQuaternionBoard::getGyrometer(gx, gy, gz);
            }

            virtual bool imuReady(void) override
            {
                return _lsm6dsm.checkNewData();
            }

            virtual void imuReadAccelGyro(void) override
            {
                _lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);

                // Negate to support board orientation
                _ax = -_ax;
                _gy = -_gy;
                _gz = -_gz;
            }

        public:

            BonadroneBoard(void) 
                : ArduinoBoard(38, true) // inverted LED signal
            {
                setLed(true);
                // Configure interrupt
                pinMode(LSM6DSM_INTERRUPT_PIN, INPUT);

                // Start I^2C
                Wire.begin(TWI_PINS_20_21);
                Wire.setClock(400000); // I2C frequency at 400 kHz  
                delay(100);

                // Start the LSM6DSM
                switch (_lsm6dsm.begin()) {

                    case LSM6DSM::ERROR_CONNECT:
                        i2cerror("no connection");
                        break;

                    case LSM6DSM::ERROR_ID:
                        i2cerror("bad ID");
                        break;

                    case LSM6DSM::ERROR_SELFTEST:
                        //i2cerror("failed self-test");
                        break;

                    case LSM6DSM::ERROR_NONE:
                        break;

                }

                delay(100);

                // Calibrate IMU on startup
                _lsm6dsm.calibrate(GYRO_BIAS, ACCEL_BIAS);

                // Clear the interrupt
                _lsm6dsm.clearInterrupt();
                setLed(false);
            }

    }; // class BonadroneBoard

    class BonadroneStandard : public BonadroneBoard {

        private:

            StandardMotor motors[4] = { 
                StandardMotor(MOTOR_PIN_1), 
                StandardMotor(MOTOR_PIN_2), 
                StandardMotor(MOTOR_PIN_3), 
                StandardMotor(MOTOR_PIN_4) 
            };


        protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                motors[index].write(value);
            }

        public:

            BonadroneStandard(void) 
                : BonadroneBoard()
            {
                for (uint8_t k=0; k<4; ++k) {
                    motors[k].init();
                }
            }

    }; // class BonadroneStandard

    class BonadroneMultiShot : public BonadroneBoard {

        private:

            MultiShotMotor motors[4] = { 
                MultiShotMotor(MOTOR_PIN_1), 
                MultiShotMotor(MOTOR_PIN_2), 
                MultiShotMotor(MOTOR_PIN_3), 
                MultiShotMotor(MOTOR_PIN_4) 
            };

        protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                motors[index].write(value);
            }


        public:

            BonadroneMultiShot(void) : BonadroneBoard()
        {
            for (uint8_t k=0; k<4; ++k) {
                motors[k].init();
            }
        }

    }; // class BonadroneMultiShot

    class BonadroneBrushed : public BonadroneBoard {

        private:

            BrushedMotor motors[4] = { 
                BrushedMotor(MOTOR_PIN_1), 
                BrushedMotor(MOTOR_PIN_2), 
                BrushedMotor(MOTOR_PIN_3), 
                BrushedMotor(MOTOR_PIN_4) 
            };

        protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                motors[index].write(value);
            }


        public:

            BonadroneBrushed(void) : BonadroneBoard()
        {
            for (int k=0; k<4; ++k) {
                motors[k].init();
            }
        }

    }; // class BonadroneBrushed

} // namespace hf
