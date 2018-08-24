/*
   Sketch for Revo board with Spektrum DSMX receiver

   Copyright (c) 2018 Simon D. Levy

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

#include <hackflight.hpp>
#include <mixers/quadx.hpp>
#include "revo.h"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static hf::Hackflight h;

extern "C" {

#include "../dsmx.h"

    static Revo * board;

    static DSMX_Receiver * rx;

    void setup(void)
    {
        rx = new DSMX_Receiver(UARTDEV_1, CHANNEL_MAP);

        rx->begin();

        board = new Revo();
    }

    void loop(void)
    {
        hf::Debug::printf("%d\n", (int)(rx->gotNewFrame()));
        board->delaySeconds(.01);
    }

} // extern "C"
