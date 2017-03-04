/*
   main routine for "HelloHackflight" demo

   Copyright (C) Simon D. Levy 2017

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

#include <stdio.h>
#include <time.h>

int main(int argc, char ** argv)
{
    Hackflight hackflight;

    hackflight.initialize();

    while (true) {
        printf("%ld\n", clock());
    }

    return 0;
}
