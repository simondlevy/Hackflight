/*
   Hackflight example with C++ PID controllers

   Copyright (C) 2024 Simon D. Levy

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

#include <hackflight.hpp>

#include <board_tinypico.hpp>

static hf::Board _board;


void setup() 
{
    _board.init();
}

void loop() 
{
    float dt=0;
    hf::demands_t demands = {};
    hf::state_t state = {};

    _board.readData(dt, demands, state);

    _board.calibrateMotors();
}
