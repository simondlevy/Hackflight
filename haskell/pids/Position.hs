{--
  X/Y position PID control algorithm for real and simulated flight controllers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Position where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

run kp dt target actual = demand where

  error = target - actual

  demand = kp * error

{--
  Demand is input as desired speed in meter per second, output as
  angles in degrees.
--}

positionController dt state demands = demands'  where

  kp = 25
    
  rollDemand = run kp dt (roll demands) (dy state)

  pitchDemand = run kp dt (pitch demands) (dx state)

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
