{--
  Altitude PID control algorithm for real and simulated flight controllers
 
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

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

{-- 

  Demand is input as normalized altitude target in meters and output as 
  climb rate in meters-per-second

--}

altitudeController target dt state demands = demands'  where

  kp = 2.0
  ki = 0.5
  ilimit = 5000

  error = target - (zz state)

  integ = constrain (integ' + error * dt) (-ilimit) ilimit

  integ' = [0] ++ integ

  thrust' = kp * error + ki * integ

  demands' = Demands thrust' (roll demands) (pitch demands) (yaw demands)
