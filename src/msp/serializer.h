/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

class MspSerializer {

    private:

        static const int OUTBUF_SIZE = 128;

        uint8_t outBufChecksum;
        uint8_t outBufIndex;

        void serialize16(int16_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
        }

        void prepareToSerialize(uint8_t type, uint8_t count, uint8_t size)
        {
            outBufSize = 0;
            outBufIndex = 0;
            outBufChecksum = 0;

            addToOutBuf('$');
            addToOutBuf('M');
            addToOutBuf('>');
            serialize8(count*size);
            serialize8(type);
        }

        void addToOutBuf(uint8_t a)
        {
            outBuf[outBufSize++] = a;
        }

        void serialize8(uint8_t a)
        {
            addToOutBuf(a);
            outBufChecksum ^= a;
        }

        void prepareToSerializeBytes(uint8_t type, uint8_t count)
        {
            prepareToSerialize(type, count, 1);
        }

        void serializeByte(uint8_t src)
        {
            serialize8(src);
        }

        void prepareToSerializeInts(uint8_t type, uint8_t count)
        {
            prepareToSerialize(type, count, 4);
        }

        void prepareToSerializeFloats(uint8_t type, uint8_t count)
        {
            prepareToSerialize(type, count, 4);
        }

    public:

        uint8_t outBuf[OUTBUF_SIZE];
        uint8_t outBufSize;

        void prepareToSerializeShorts(uint8_t messageType, uint8_t count)
        {
            prepareToSerialize(messageType, count, 2);
        }

        void completeSerialize(void)
        {
            serialize8(outBufChecksum);
        }

        void serializeShort(uint16_t src)
        {
            uint16_t a;
            memcpy(&a, &src, 2);
            serialize16(a);
        }

        void serializeShorts(uint8_t messageType, int16_t src[], uint8_t count)
        {
            prepareToSerializeShorts(messageType, count);

            for (auto k=0; k<count; ++k) {
                serializeShort(src[k]);
            }

            completeSerialize();
        }

}; // class MspSerializer
