/*
   Multiwii Serial Protocol parsing support for Hackflight

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
#include <string.h>

#include <hackflight.h>

namespace hf {

    class MspParser {

        public:

            /**
             * Returns message ID or 0 for not  ready
             */
            uint8_t parse(const uint8_t byte)
            {
                uint8_t result = 0;

                switch (state) {

                    case 0:
                        if (byte == '$') {  // $
                            state = 1;
                        }
                        break;

                    case 1:
                        if (byte == 'M') { // M
                            state = 2;
                        }
                        else {  // restart and try again
                            state = 0;
                        }
                        break;

                    case 2:
                        state = 3;
                        break;

                    case 3:
                        length_expected = byte;
                        checksum = byte;
                        index = 0;
                        state = 4;
                        break;

                    case 4:
                        id = byte;
                        length_received = 0;
                        checksum ^= byte;
                        if (length_expected > 0) {
                            // process payload
                            state = 5;
                        }
                        else {
                            // no payload
                            state = 6;
                        }
                        break;

                    case 5:
                        buffer[index++] = byte;
                        checksum ^= byte;
                        length_received++;
                        if (length_received >= length_expected) {
                            state = 6;
                        }
                        break;

                    case 6:

                        if (checksum == byte) {
                            result = id;
                        }
                        state = 0;

                        break;
                }

                return result;
            }

            uint16_t getUshort(const uint8_t index)
            {
                const uint8_t offset = 2 * index;
                uint16_t value = (buffer[offset+1] << 8) | buffer[offset];
                return value;
            }

        private:

            uint8_t state;
            uint8_t buffer[256];
            uint8_t length_expected;
            uint8_t length_received;
            uint8_t checksum;
            uint8_t index;
            uint8_t id;

    };

}
