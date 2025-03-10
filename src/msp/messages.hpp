/*
   Multiwii Serial Protocol (MSP) message IDs.  

   See http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol

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

namespace hf {

    static const uint8_t MSP_SPIKES = 121;
    static const uint8_t MSP_STATE = 122;
    static const uint8_t MSP_SET_RC = 200;
}
