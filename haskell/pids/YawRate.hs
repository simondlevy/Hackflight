{--
  Yaw rate PID control algorithm for real and simulated flight controllers
 
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

module YawRate where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

{--
   Demand is input in degrees per second and output in units appropriate
   for our motors, both nose-right positive.
--}

yawRateController dt state demands = demands' where

    kp = 1.20e-2

    error = (yaw demands) - (dpsi state)

    yaw' = kp * error

    demands' = Demands (thrust demands) (roll demands) (pitch demands) yaw'

