/*
   Multiwii Serial Protocol serialization support for Hackflight

   Copyright (c) 2025 Simon D. Levy

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
#include <array>

class MspSerializer {

    private:

        typedef std::array<uint8_t, 256> payload_t;

    public:

        MspSerializer() = default;

        MspSerializer& operator=(const MspSerializer& other) = default;

        MspSerializer(
                const payload_t payload,
                const uint8_t payloadSize,
                const uint8_t payloadChecksum,
                const uint8_t payloadIndex)
            :
                _payload(payload),
                _payloadSize(payloadSize),
                _payloadChecksum(payloadChecksum),
                _payloadIndex(payloadIndex) {}

        void serializeBytes(
                const uint8_t messageType, const uint8_t src[], const uint8_t count)
        {
            prepareToSerializeBytes(messageType, count);

            for (auto k=0; k<count; ++k) {
                serializeByte(src[k]);
            }

            completeSerialize();
        }

        void serializeFloats(
                const uint8_t messageType, const float src[], const uint8_t count)
        {
            prepareToSerializeFloats(messageType, count);

            for (auto k=0; k<count; ++k) {
                serializeFloat(src[k]);
            }

            completeSerialize();
        }

        void serializeShorts(
                const uint8_t messageType, const int16_t src[], const uint8_t count)
        {
            prepareToSerializeShorts(messageType, count);

            for (auto k=0; k<count; ++k) {
                serializeShort(src[k]);
            }

            completeSerialize();
        }

        auto payloadData() -> uint8_t *
        {
            return _payload.data();
        }

        auto payloadSize() -> uint8_t
        {
            return _payloadSize;
        }

    private:

        payload_t _payload;
        uint8_t _payloadSize;
        uint8_t _payloadChecksum;
        uint8_t _payloadIndex;

        void serialize32(const int32_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
            serialize8((a >> 16) & 0xFF);
            serialize8((a >> 24) & 0xFF);
        }

        void serialize16(const int16_t a)
        {
            serialize8(a & 0xFF);
            serialize8((a >> 8) & 0xFF);
        }

        void serialize8(const uint8_t a)
        {
            addToOutBuf(a);
            _payloadChecksum ^= a;
        }

        void prepareToSerialize(
                const uint8_t type, const uint8_t count, const uint8_t size)
        {
            _payloadSize = 0;
            _payloadIndex = 0;
            _payloadChecksum = 0;

            addToOutBuf('$');
            addToOutBuf('M');
            addToOutBuf('>');
            serialize8(count*size);
            serialize8(type);
        }

        void addToOutBuf(const uint8_t a)
        {
            _payload[_payloadSize++] = a;
        }

        static auto addToOutBuf(
                const MspSerializer & s, const uint8_t a) -> MspSerializer
        {
            auto payload = s._payload;

            payload[s._payloadSize] = a;

            return MspSerializer(
                    payload,
                    s._payloadSize + 1,
                    s._payloadChecksum,
                    s._payloadIndex);
        }

        void prepareToSerializeBytes(const uint8_t type, const uint8_t count)
        {
            prepareToSerialize(type, count, 1);
        }

        void serializeByte(const uint8_t src)
        {
            serialize8(src);
        }

        void prepareToSerializeInts(const uint8_t msgtype, const uint8_t count)
        {
            prepareToSerialize(msgtype, count, 4);
        }

        void prepareToSerializeFloats(const uint8_t msgtype, const uint8_t count)
        {
            prepareToSerialize(msgtype, count, 4);
        }

        void prepareToSerializeShorts(const uint8_t msgtype, const uint8_t count)
        {
            prepareToSerialize(msgtype, count, 2);
        }

        void completeSerialize(void)
        {
            serialize8(_payloadChecksum);
            _payloadIndex = 0;
        }

        void serializeFloat(const float src)
        {
            uint32_t a;
            memcpy(&a, &src, 4);
            serialize32(a);
        }

        void serializeShort(const uint16_t src)
        {
            int16_t a;
            memcpy(&a, &src, 2);
            serialize16(a);
        }
};
