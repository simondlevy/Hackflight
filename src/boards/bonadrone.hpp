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
#include "softquat.hpp"
#include "arduino.hpp"

namespace hf {

    class Bonadrone : public ArduinoBoard, public SoftwareQuaternionBoard {

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

            const uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};

            virtual void writeMotor(uint8_t index, float value) = 0;

            virtual bool  getQuaternion(float quat[4]) override 
            {
                return SoftwareQuaternionBoard::getQuaternion(quat, getTime());
            }

            virtual bool  getGyrometer(float gyroRates[3]) override
            {
                return SoftwareQuaternionBoard::getGyrometer(gyroRates);
            }

            bool imuRead(void)
            {
                if (_lsm6dsm.checkNewData()) {

                    _lsm6dsm.readData(_ax, _ay, _az, _gx, _gy, _gz);

                    // Negate to support board orientation
                    _ax = -_ax;
                    _gy = -_gy;
                    _gz = -_gz;

                    return true;

                } 

                return false;
            }

        public:

            Bonadrone(void) : ArduinoBoard(38, true) // inverted LED signal
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

    }; // class Bonadrone

    class BonadroneStandard : public Bonadrone {

        private:

            // Min, max PWM values
            const uint16_t PWM_MIN = 1000;
            const uint16_t PWM_MAX = 2000;

        protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                analogWrite(MOTOR_PINS[index], (uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)) >> 3);
            }

        public:

            BonadroneStandard(void) : Bonadrone()
            {
                for (uint8_t k=0; k<4; ++k) {
                    pinMode(MOTOR_PINS[k], OUTPUT);
                    analogWrite(MOTOR_PINS[k], PWM_MIN>>3);
                }
            }

    }; // class BonadroneStandard

    class BonadroneMultiShot : public Bonadrone {

        private:

            // Min, max PWM values
            const uint16_t PWM_MIN = 100;
            const uint16_t PWM_MAX = 500;

        protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                value = int(value*100)/100.0;
                analogWrite(MOTOR_PINS[index], (uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)));
            }


        public:

            BonadroneMultiShot(void) : Bonadrone()
            {
                for (uint8_t k=0; k<4; ++k) {
                    analogWriteFrequency(MOTOR_PINS[k], 2000);
                    analogWriteRange(MOTOR_PINS[k], 10000);
                    analogWrite(MOTOR_PINS[k], PWM_MIN);
                }
            }

    }; // class BonadroneMultiShot

    class BonadroneBrushed : public Bonadrone {

        protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                analogWrite(MOTOR_PINS[index], (uint8_t)(value * 255));
            }


        public:

            BonadroneBrushed(void) : Bonadrone()
        {
            for (int k=0; k<4; ++k) {
                analogWriteFrequency(MOTOR_PINS[k], 10000);  
                analogWrite(MOTOR_PINS[k], 0);  
            }
        }

    }; // class BonadroneBrushed

} // namespace hf
