/*
   debug.cpp : Generic debugging code for Hackflight source
   
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

#include <sensors/linalg.hpp>

#include <stdio.h>

void hf::Board::outbuf(char * buf)
{
    printf("%s", buf);
}

int main(int argc, char ** argv)
{
    float v[9] = {1,2,3,4,5,6,7,8,9};

    hf::Matrix a = hf::Matrix(3,3,v);

    hf::Matrix at(3,3);

    hf::Matrix::trans(a, at);

    at.dump();
}
