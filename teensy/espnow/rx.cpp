/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <firmware/rx.hpp>
#include <firmware/msp/parser.hpp>
using namespace hf;

static RX::Data _data;

static MspParser _parser;

void serialEvent1()
{
    while (Serial1.available()) {

        const auto msgid = _parser.parse(Serial1.read());
    }
}

void RX::begin(HardwareSerial * serial)
{
    serial->begin(115200);
}

auto RX::read() -> RX::Data
{
    return RX::Data();
}
