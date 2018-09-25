/*
   mspserializer.hpp: header-only implementation of MSP serializing routines

   Auto-generated code: DO NOT EDIT!

   Copyright (C) Simon D. Levy 2018

   This program is part of Hackflight

   This code is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as 
   published by the Free Software Foundation, either version 3 of the 
   License, or (at your option) any later version.

   This code is distributed in the hope that it will be useful,     
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License 
   along with this code.  If not, see <http:#www.gnu.org/licenses/>.
 */


#pragma once

#include <stdint.h>

namespace hf {

    class MspSerializer {

        private:

            static uint8_t CRC8(uint8_t * data, int n) 
            {
                uint8_t crc = 0x00;

                for (int k=0; k<n; ++k) {

                    crc ^= data[k];
                }

                return crc;
            }

        public:

            static const uint8_t MAXLEN = 255;

            static uint8_t serialize_GET_ALTITUDE_METERS_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 123;
                bytes[5] = 123;

                return 6;
            }

            static uint8_t serialize_GET_ALTITUDE_METERS(uint8_t bytes[], float  estalt, float  vario)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 8;
                bytes[4] = 123;

                memcpy(&bytes[5], &estalt, sizeof(float));
                memcpy(&bytes[9], &vario, sizeof(float));

                bytes[13] = CRC8(&bytes[3], 10);

                return 14;
            }

            static uint8_t serialize_GET_LOITER_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 126;
                bytes[5] = 126;

                return 6;
            }

            static uint8_t serialize_GET_LOITER(uint8_t bytes[], float  agl, float  flowx, float  flowy)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 12;
                bytes[4] = 126;

                memcpy(&bytes[5], &agl, sizeof(float));
                memcpy(&bytes[9], &flowx, sizeof(float));
                memcpy(&bytes[13], &flowy, sizeof(float));

                bytes[17] = CRC8(&bytes[3], 14);

                return 18;
            }

            static uint8_t serialize_SET_MOTOR_NORMAL(uint8_t bytes[], float  m1, float  m2, float  m3, float  m4)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 16;
                bytes[4] = 215;

                memcpy(&bytes[5], &m1, sizeof(float));
                memcpy(&bytes[9], &m2, sizeof(float));
                memcpy(&bytes[13], &m3, sizeof(float));
                memcpy(&bytes[17], &m4, sizeof(float));

                bytes[21] = CRC8(&bytes[3], 18);

                return 22;
            }

            static uint8_t serialize_SET_ARMED(uint8_t bytes[], uint8_t  flag)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 1;
                bytes[4] = 216;

                memcpy(&bytes[5], &flag, sizeof(uint8_t));

                bytes[6] = CRC8(&bytes[3], 3);

                return 7;
            }

            static uint8_t serialize_GET_ATTITUDE_RADIANS_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 122;
                bytes[5] = 122;

                return 6;
            }

            static uint8_t serialize_GET_ATTITUDE_RADIANS(uint8_t bytes[], float  roll, float  pitch, float  yaw)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 12;
                bytes[4] = 122;

                memcpy(&bytes[5], &roll, sizeof(float));
                memcpy(&bytes[9], &pitch, sizeof(float));
                memcpy(&bytes[13], &yaw, sizeof(float));

                bytes[17] = CRC8(&bytes[3], 14);

                return 18;
            }

            static uint8_t serialize_GET_RC_NORMAL_Request(uint8_t bytes[])
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 60;
                bytes[3] = 0;
                bytes[4] = 121;
                bytes[5] = 121;

                return 6;
            }

            static uint8_t serialize_GET_RC_NORMAL(uint8_t bytes[], float  c1, float  c2, float  c3, float  c4, float  c5, float  c6)
            {
                bytes[0] = 36;
                bytes[1] = 77;
                bytes[2] = 62;
                bytes[3] = 24;
                bytes[4] = 121;

                memcpy(&bytes[5], &c1, sizeof(float));
                memcpy(&bytes[9], &c2, sizeof(float));
                memcpy(&bytes[13], &c3, sizeof(float));
                memcpy(&bytes[17], &c4, sizeof(float));
                memcpy(&bytes[21], &c5, sizeof(float));
                memcpy(&bytes[25], &c6, sizeof(float));

                bytes[29] = CRC8(&bytes[3], 26);

                return 30;
            }

    }; // class MspSerializer

} // namespace hf
