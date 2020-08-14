/*
   ESP32 Arduino code for DSHOT600 protocol

   Copyright (c) 2020 Simon D. Levy

   Adapted from https://github.com/JyeSmith/dshot-esc-tester/blob/master/dshot-esc-tester.ino, 
   which contains the following licensing notice:

   "THE PROP-WARE LICENSE" (Revision 42):
   <https://github.com/JyeSmith> wrote this file.  As long as you retain this notice you
   can do whatever you want with this stuff. If we meet some day, and you think
   this stuff is worth it, you can buy me some props in return.   Jye Smith

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

#include <stdint.h>
#include <stdarg.h>

#include "esp32-hal.h"

namespace hf {

    class Esp32DShot600 {

        private:

            static const uint8_t MAX_MOTORS = 10;

            // XXX idle value should be calibrated for each motor
            static constexpr uint16_t MIN = 48;
            static constexpr uint16_t MAX = 2047;

            typedef struct {

                rmt_data_t dshotPacket[16];
                rmt_obj_t * rmt_send;
                uint16_t outputValue;
                bool requestTelemetry;
                uint8_t receivedBytes;
                uint8_t pin;

            } motor_t;

            motor_t _motors[MAX_MOTORS] = {};

            uint8_t _motorCount = 0;

            static void coreTask(void * params)
            {

                Esp32DShot600 * dshot = (Esp32DShot600 *)params;

                while (true) {

                    for (uint8_t k=0; k<dshot->_motorCount; ++k) {
                        dshot->outputOne(&dshot->_motors[k]);
                    }

                    delay(1);
                } 
            }

            void outputOne(motor_t * motor)
            {
                uint16_t packet = (motor->outputValue << 1) /* | (motor->telemetry ? 1 : 0) */ ;

                // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
                int csum = 0;
                int csum_data = packet;
                for (int i = 0; i < 3; i++) {
                    csum ^=  csum_data;
                    csum_data >>= 4;
                }
                csum &= 0xf;
                packet = (packet << 4) | csum;

                // durations are for dshot600
                // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
                // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
                // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
                // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
                for (int i = 0; i < 16; i++) {
                    if (packet & 0x8000) {
                        motor->dshotPacket[i].level0 = 1;
                        motor->dshotPacket[i].duration0 = 100;
                        motor->dshotPacket[i].level1 = 0;
                        motor->dshotPacket[i].duration1 = 34;
                    } else {
                        motor->dshotPacket[i].level0 = 1;
                        motor->dshotPacket[i].duration0 = 50;
                        motor->dshotPacket[i].level1 = 0;
                        motor->dshotPacket[i].duration1 = 84;
                    }
                    packet <<= 1;
                }

                rmtWrite(motor->rmt_send, motor->dshotPacket, 16);

            } // outputOne

        public:

            Esp32DShot600(void)
            {
                _motorCount = 0;
            }

            void addMotor(uint8_t pin)
            {
                _motors[_motorCount++].pin = pin;
            }

            bool begin(void)
            {
                for (uint8_t k=0; k<_motorCount; ++k) {

                    motor_t * motor = &_motors[k];

                    if ((motor->rmt_send = rmtInit(motor->pin, true, RMT_MEM_64)) == NULL) {
                        return false;
                    }

                    rmtSetTick(motor->rmt_send, 12.5); // 12.5ns sample rate

                    // Output disarm signal while esc initialises
                    motor->outputValue = MIN;
                    while (millis() < 3500) {
                        outputOne(motor);
                        delay(1);  
                    }
                }

                TaskHandle_t Task;
                xTaskCreatePinnedToCore(coreTask, "Task", 10000, this, 1, &Task, 0); 

                return true;
            }

            void writeMotor(uint8_t index, float value)
            {
                _motors[index].outputValue = MIN + (uint16_t)(value * (MAX-MIN));
            }

    }; // class Esp32DShot600

    class NewEsp32DShot600 {

        private:

            static const uint8_t MAX_MOTORS = 10;

            // XXX idle value should be calibrated for each motor
            static constexpr uint16_t MIN = 48;
            static constexpr uint16_t MAX = 2047;

            typedef struct {

                rmt_data_t dshotPacket[16];
                rmt_obj_t * rmt_send;
                uint16_t outputValue;
                bool requestTelemetry;
                uint8_t receivedBytes;
                uint8_t pin;

            } motor_t;

            motor_t _motors[MAX_MOTORS] = {};

            uint8_t _motorCount = 0;

            static void coreTask(void * params)
            {

                NewEsp32DShot600 * dshot = (NewEsp32DShot600 *)params;

                while (true) {

                    for (uint8_t k=0; k<dshot->_motorCount; ++k) {
                        dshot->outputOne(&dshot->_motors[k]);
                    }

                    delay(1);
                } 
            }

            void outputOne(motor_t * motor)
            {
                uint16_t packet = (motor->outputValue << 1) /* | (motor->telemetry ? 1 : 0) */ ;

                // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
                int csum = 0;
                int csum_data = packet;
                for (int i = 0; i < 3; i++) {
                    csum ^=  csum_data;
                    csum_data >>= 4;
                }
                csum &= 0xf;
                packet = (packet << 4) | csum;

                // durations are for dshot600
                // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
                // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
                // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
                // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
                for (int i = 0; i < 16; i++) {
                    if (packet & 0x8000) {
                        motor->dshotPacket[i].level0 = 1;
                        motor->dshotPacket[i].duration0 = 100;
                        motor->dshotPacket[i].level1 = 0;
                        motor->dshotPacket[i].duration1 = 34;
                    } else {
                        motor->dshotPacket[i].level0 = 1;
                        motor->dshotPacket[i].duration0 = 50;
                        motor->dshotPacket[i].level1 = 0;
                        motor->dshotPacket[i].duration1 = 84;
                    }
                    packet <<= 1;
                }

                rmtWrite(motor->rmt_send, motor->dshotPacket, 16);

            } // outputOne

        public:

            NewEsp32DShot600(void)
            {
                _motorCount = 0;
            }

            void addMotor(uint8_t pin)
            {
                _motors[_motorCount++].pin = pin;
            }

            bool begin(void)
            {
                for (uint8_t k=0; k<_motorCount; ++k) {

                    motor_t * motor = &_motors[k];

                    if ((motor->rmt_send = rmtInit(motor->pin, true, RMT_MEM_64)) == NULL) {
                        return false;
                    }

                    rmtSetTick(motor->rmt_send, 12.5); // 12.5ns sample rate

                    // Output disarm signal while esc initialises
                    motor->outputValue = MIN;
                    while (millis() < 3500) {
                        outputOne(motor);
                        delay(1);  
                    }
                }

                TaskHandle_t Task;
                xTaskCreatePinnedToCore(coreTask, "Task", 10000, this, 1, &Task, 0); 

                return true;
            }

            void writeMotor(uint8_t index, float value)
            {
                _motors[index].outputValue = MIN + (uint16_t)(value * (MAX-MIN));
            }

    }; // class NewEsp32DShot600

} // namespace hf
