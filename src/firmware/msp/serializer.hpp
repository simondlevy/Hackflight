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

namespace hf {

    class MspSerializer {

        private:

            typedef std::array<uint8_t, 256> Payload;

        public:

            MspSerializer() = default;

            MspSerializer& operator=(const MspSerializer& other) = default;

            MspSerializer(
                    const Payload & payload,
                    const uint8_t payloadSize) 
                :
                    payload_(payload),
                    payload_size_(payloadSize) {}

            static auto serializeFloats(
                    const MspSerializer & serializer,
                    const uint8_t id,
                    const float src[],
                    const uint8_t count)
            {
                Payload payload = {};

                uint8_t checksum = 0;

                prepareToSerialize(id, count, 4, payload, checksum);

                for (uint8_t k=0; k<count; ++k) {
                    serializeFloat(5 + k*4, src[k], payload, checksum);
                }

                payload[5 + 4 * count] = checksum;

                return MspSerializer(payload, 6 + 4 * count);
            }

            static auto payloadBytes(
                    const MspSerializer & serializer) -> uint8_t *
            {
                return (uint8_t *)serializer.payload_.data();
            }

            static auto payloadSize(
                    const MspSerializer & serializer) -> uint8_t
            {
                return serializer.payload_size_;
            }

        private:

            Payload payload_;

            uint8_t payload_size_;

            static void serialize32(
                    const uint8_t k,
                    const int32_t a,
                    Payload & payload,
                    uint8_t & checksum)
            {
                serialize8(a & 0xFF, k, payload, checksum);
                serialize8((a >> 8) & 0xFF, k+1, payload, checksum);
                serialize8((a >> 16) & 0xFF, k+2, payload, checksum);
                serialize8((a >> 24) & 0xFF, k+3, payload, checksum);
            }

            static void serialize8(
                    const uint8_t a,
                    const uint8_t k,
                    Payload & payload,
                    uint8_t &checksum)
            {
                payload[k] = a;
                checksum ^= a;
            }

            static void prepareToSerialize(
                    const uint8_t id,
                    const uint8_t count,
                    const uint8_t size,
                    Payload & payload,
                    uint8_t & checksum)
            {
                payload[0] = '$';
                payload[1] = 'M';
                payload[2] = '>';

                serialize8(count*size, 3, payload, checksum);

                serialize8(id, 4, payload, checksum);
            }

            static void serializeFloat(
                    const int k,
                    const float src,
                    Payload & payload,
                    uint8_t & checksum)
            {
                uint32_t a=0;
                memcpy(&a, &src, 4);
                serialize32(k, a, payload, checksum);
            }
    };
}
