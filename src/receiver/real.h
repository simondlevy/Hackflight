/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along
   with Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdlib.h>

#include "receiver.h"
#include "utils.h"


class RealReceiver : public Receiver {

    protected:

        virtual void parse(const uint8_t c) = 0;

    public:

        void handleSerialEvent(HardwareSerial & serial) {

            while (serial.available()) {
                parse(serial.read());
            }
        }

}; // class RealReceiver

