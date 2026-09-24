/*
   Hackflight flight-controller sketch for Teensy quadcopter using ESP32
   receiver with MSP protocol

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

#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

static int16_t thr_;
static int16_t rol_;
static int16_t pit_;
static int16_t yaw_;
static int16_t arm_;
static int16_t hov_;
static int16_t aut_;

void serialEvent3()
{
    static hf::MspParser parser_;

    while (Serial3.available()) {

        parser_ = hf::MspParser::Parse(parser_, Serial3.read());

        if (hf::MspParser::GetId(parser_) == kMspSetChannels) {

            thr_ = hf::MspParser::GetShort(parser_, 0);
            rol_ = hf::MspParser::GetShort(parser_, 1);
            pit_ = hf::MspParser::GetShort(parser_, 2);
            yaw_ = hf::MspParser::GetShort(parser_, 3);
            arm_ = hf::MspParser::GetShort(parser_, 4);
            hov_ = hf::MspParser::GetShort(parser_, 5);
            aut_ = hf::MspParser::GetShort(parser_, 6);
        }
    }
}

static auto AxisToFloat(const uint16_t val) -> float
{
    return map((float)val, 0, 4095, -1.f, +1.f);
}

void setup()
{
    Serial3.begin(115200);
}

void loop()
{
    printf("t=%+0.3f r=%+0.3f p=%+0.3f y=%+0.3f | arm=%d hov=%d aut=%d\n",
            AxisToFloat(thr_),
            AxisToFloat(rol_),
            AxisToFloat(pit_),
            AxisToFloat(yaw_),
            arm_, hov_, aut_);

    delay(10);
}
