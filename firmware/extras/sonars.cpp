/*
   sonars.cpp : Sonars class implementation

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

#ifdef __arm__
extern "C" {
#endif

#include "hackflight.hpp"

void Sonars::init(void)
{
    this->avail = true;
    this->count = 5;
    for (int k=0; k<this->count; ++k) {
        //this->avail = this->avail && Board::sonarInit(k);
        this->distances[k] = 0;
    }
    this->index = 0;
    this->ready = false;
}

bool Sonars::available(void)
{
    return this->avail;
}

uint16_t Sonars::getAltitude(void)
{
    return this->distances[1]; // Back, Bottom, Front, Left, Right
}

void Sonars::update(void)
{
    // Update hardware
    Board::sonarUpdate(this->index);

    this->distances[this->index] = Board::sonarGetDistance(this->index);

    // Address hardware cyclically
    this->index = (this->index + 1) % this->count;

    // As soon as we've read all sonars, we're ready to report their values
    if (this->index == this->count-1)
        this->ready = true;
}

#ifdef __arm__
} // extern "C"
#endif
