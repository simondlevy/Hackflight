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

    class MspMessage {

        friend class MspParser;

        protected:

        static const int MAXBUF = 256;

        uint8_t _bytes[MAXBUF];
        int _pos;
        int _len;

        public:

        uint8_t start(void) 
        {
            _pos = 0;
            return getNext();
        }

        bool hasNext(void) 
        {
            return _pos <= _len;
        }


        uint8_t getNext(void) 
        {
            return _bytes[_pos++];
        }

    };

    class MspSerializer {

        public:

            MspMessage serialize_GET_RC_NORMAL_Request()
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 60;
                msg._bytes[3] = 0;
                msg._bytes[4] = 121;
                msg._bytes[5] = 121;

                msg._len = 6;

                return msg;
            }

            MspMessage serialize_GET_RC_NORMAL(float  c1, float  c2, float  c3, float  c4, float  c5, float  c6)
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 62;
                msg._bytes[3] = 24;
                msg._bytes[4] = 121;

                memcpy(&msg._bytes[5], &c1, sizeof(float));
                memcpy(&msg._bytes[9], &c2, sizeof(float));
                memcpy(&msg._bytes[13], &c3, sizeof(float));
                memcpy(&msg._bytes[17], &c4, sizeof(float));
                memcpy(&msg._bytes[21], &c5, sizeof(float));
                memcpy(&msg._bytes[25], &c6, sizeof(float));

                msg._bytes[29] = CRC8(&msg._bytes[3], 26);

                msg._len = 30;

                return msg;
            }

            MspMessage serialize_GET_ATTITUDE_RADIANS_Request()
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 60;
                msg._bytes[3] = 0;
                msg._bytes[4] = 122;
                msg._bytes[5] = 122;

                msg._len = 6;

                return msg;
            }

            MspMessage serialize_GET_ATTITUDE_RADIANS(float  roll, float  pitch, float  yaw)
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 62;
                msg._bytes[3] = 12;
                msg._bytes[4] = 122;

                memcpy(&msg._bytes[5], &roll, sizeof(float));
                memcpy(&msg._bytes[9], &pitch, sizeof(float));
                memcpy(&msg._bytes[13], &yaw, sizeof(float));

                msg._bytes[17] = CRC8(&msg._bytes[3], 14);

                msg._len = 18;

                return msg;
            }

            MspMessage serialize_GET_ALTITUDE_METERS_Request()
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 60;
                msg._bytes[3] = 0;
                msg._bytes[4] = 123;
                msg._bytes[5] = 123;

                msg._len = 6;

                return msg;
            }

            MspMessage serialize_GET_ALTITUDE_METERS(float  estalt, float  vario)
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 62;
                msg._bytes[3] = 8;
                msg._bytes[4] = 123;

                memcpy(&msg._bytes[5], &estalt, sizeof(float));
                memcpy(&msg._bytes[9], &vario, sizeof(float));

                msg._bytes[13] = CRC8(&msg._bytes[3], 10);

                msg._len = 14;

                return msg;
            }

            MspMessage serialize_GET_LOITER_RAW_Request()
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 60;
                msg._bytes[3] = 0;
                msg._bytes[4] = 126;
                msg._bytes[5] = 126;

                msg._len = 6;

                return msg;
            }

            MspMessage serialize_GET_LOITER_RAW(uint8_t  agl, uint8_t  flowx, uint8_t  flowy)
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 62;
                msg._bytes[3] = 3;
                msg._bytes[4] = 126;

                memcpy(&msg._bytes[5], &agl, sizeof(uint8_t));
                memcpy(&msg._bytes[6], &flowx, sizeof(uint8_t));
                memcpy(&msg._bytes[7], &flowy, sizeof(uint8_t));

                msg._bytes[8] = CRC8(&msg._bytes[3], 5);

                msg._len = 9;

                return msg;
            }

            MspMessage serialize_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4)
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 62;
                msg._bytes[3] = 16;
                msg._bytes[4] = 215;

                memcpy(&msg._bytes[5], &m1, sizeof(float));
                memcpy(&msg._bytes[9], &m2, sizeof(float));
                memcpy(&msg._bytes[13], &m3, sizeof(float));
                memcpy(&msg._bytes[17], &m4, sizeof(float));

                msg._bytes[21] = CRC8(&msg._bytes[3], 18);

                msg._len = 22;

                return msg;
            }

            MspMessage serialize_SET_ARMED(uint8_t  flag)
            {
                MspMessage msg;

                msg._bytes[0] = 36;
                msg._bytes[1] = 77;
                msg._bytes[2] = 62;
                msg._bytes[3] = 1;
                msg._bytes[4] = 216;

                memcpy(&msg._bytes[5], &flag, sizeof(uint8_t));

                msg._bytes[6] = CRC8(&msg._bytes[3], 3);

                msg._len = 7;

                return msg;
            }

    }; // class MspSerializer

} // namespace hf
