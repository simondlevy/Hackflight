/*
   simtest.cpp : Lightweight (text-only) simulator for Hackflight

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


#include <math.h>

#include <hackflight.hpp>
#include <models/3dfly.hpp> // arbitrary
#include <receivers/sim/sim.hpp>
#include <boards/sim/linux.hpp>

int main(int argc, char ** argv)
{
	hf::Hackflight hackflight;
	hf::SimBoardLinux  board;
	hf::Controller controller;
	hf::ThreeDFly  model;

	hackflight.init(&board, &controller, &model);

    while (true) {

        hackflight.update();
    }
	
    return 0;
}
