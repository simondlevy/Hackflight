{--
  Climb-rate algorithm simulated flight controllers
 
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

module ClimbRate where

import Language.Copilot
import Copilot.Compile.C99

import Pid
import Demands
import State
import Utils

{-- 

  Demand is input as climb rate in meters per second and output as arbitrary
  positive value to be scaled according to motor characteristics.

--}

climbRateController hitTakeoffButton time state demands = demands'

  where

    kp = 2

    thrust_takeoff = 80 :: SFloat -- rad /sec

    thrust_base = 74.565 :: SFloat -- rad /sec

    takeoff_time = 3.0 :: SFloat -- sec

    thrust' = if time > takeoff_time
              then thrust_base + kp * ((thrust demands) - (dz state))
              else if hitTakeoffButton 
              then thrust_takeoff
              else 0

    demands' = Demands thrust' (roll demands) (pitch demands) (yaw demands)
