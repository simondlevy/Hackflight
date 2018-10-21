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

namespace hf {

    // Interrupt support 

    static bool gotNewAccelGyroData;
    static void lsm6dsmInterruptHandler()
    {
        gotNewAccelGyroData = true;
    }

    class Bonadrone : public SoftwareQuaternionBoard {

        private:

            const uint8_t LED_PIN = 38;

            // LSM6DSM data-ready interrupt pin
            const uint8_t LSM6DSM_INTERRUPT_PIN = 2;

            
            // Paramters to experiment with ------------------------------------------------------------------------

            // LSM6DSM full-scale settings
            static const LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_2G;
            static const LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_245DPS;
            static const LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_833Hz;
            static const LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_833Hz;

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

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void setLed(bool isOn)
            { 
                digitalWrite(LED_PIN, isOn ? LOW : HIGH);
            }

            uint8_t serialAvailableBytes(void)
            {
                return Serial.available();
            }

            uint8_t serialReadByte(void)
            {
                return Serial.read();
            }

            void serialWriteByte(uint8_t c)
            {
                Serial.write(c);
            }

            virtual uint32_t getMicroseconds(void) override
            {
                return micros();
            }

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }

            bool imuRead(void)
            {
                if (gotNewAccelGyroData) {

                    gotNewAccelGyroData = false;

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

            Bonadrone(void)
            {
                // Begin serial comms
                Serial.begin(115200);

                // Setup LED pin and turn it off
                pinMode(LED_PIN, OUTPUT);
                digitalWrite(LED_PIN, HIGH);

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

                attachInterrupt(LSM6DSM_INTERRUPT_PIN, lsm6dsmInterruptHandler, RISING);  

                // Clear the interrupt
                _lsm6dsm.clearInterrupt();

                // Do general real-board initialization
                RealBoard::init();
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

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
