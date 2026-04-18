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

            typedef std::array<uint8_t, 256> payload_t;

        public:

            MspSerializer() = default;

            MspSerializer& operator=(const MspSerializer& other) = default;

            void serializeFloats(
                    const uint8_t id, const float src[], const uint8_t count)
            {
                prepareToSerialize(id, count, 4);

                uint8_t k = 0;

                while (k++ < count) {
                    serializeFloat(5 + k*4, src[k]);
                }

                completeSerialize(5 + 4 * count);

                _payloadSize = 6 + 4 * count;
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

            void serialize32(const uint8_t k, const int32_t a)
            {
                serialize8(a & 0xFF, k);
                serialize8((a >> 8) & 0xFF, k+1);
                serialize8((a >> 16) & 0xFF, k+2);
                serialize8((a >> 24) & 0xFF, k+3);
            }

            /*
            void serialize8(const uint8_t a)
            {
                addToOutBuf(a);
                _payloadChecksum ^= a;
            }*/

            void serialize8(const uint8_t a, const uint8_t k)
            {
                _payload[k] = a;
                _payloadChecksum ^= a;
            }

            void prepareToSerialize(
                    const uint8_t id, const uint8_t count, const uint8_t size)
            {
                _payload[0] = '$';
                _payload[1] = 'M';
                _payload[2] = '>';

                _payloadChecksum = 0;

                serialize8(count*size, 3);

                serialize8(id, 4);

                _payloadSize = 5;
            }

            void addToOutBuf(const uint8_t a)
            {
                _payload[_payloadSize++] = a;
            }

            void completeSerialize(const uint8_t k)
            {
                serialize8(_payloadChecksum, k);
            }

            void serializeFloat(const int k, const float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(k, a);
            }
    };
}
