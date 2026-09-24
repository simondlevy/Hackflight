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

static int16_t chan1_;
static int16_t chan2_;
static int16_t chan3_;
static int16_t chan4_;
static int16_t chan5_;
static int16_t chan6_;
static int16_t chan7_;

void serialEvent3()
{
    static hf::MspParser parser_;

    while (Serial3.available()) {

        parser_ = hf::MspParser::Parse(parser_, Serial3.read());

        if (hf::MspParser::GetId(parser_) == kMspSetChannels) {

            chan1_ = hf::MspParser::GetShort(parser_, 0);
            chan2_ = hf::MspParser::GetShort(parser_, 1);
            chan3_ = hf::MspParser::GetShort(parser_, 2);
            chan4_ = hf::MspParser::GetShort(parser_, 3);
            chan5_ = hf::MspParser::GetShort(parser_, 4);
            chan6_ = hf::MspParser::GetShort(parser_, 5);
            chan7_ = hf::MspParser::GetShort(parser_, 6);
        }
    }
}

void setup()
{
    Serial3.begin(115200);
}

void loop()
{
    printf("c1=%+05d c2=%+05d c3=%+05d c4=%+05d "
            "c5=%+05d c6=%+05d c7=%+05d\n", 
            chan1_, chan2_, chan3_, chan4_, chan5_, chan6_, chan7_);

    delay(10);

}
