{--
  Yaw angle PID control algorithm for real and simulated flight controllers
 
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

module YawAngle where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

cap angle = angle''

  where angle' = if angle > 180 then angle - 360 else angle

        angle'' = if angle' < (-180) then angle' + 360 else angle'

------------------------------------------------------------------------------

-- Demand is input as desired angle normalized to [-1,+1] and output
-- as degrees per second, both nose-right positive.

yawAngleController dt state demands = demands'

    where 

      kp = 6
      kd = 0.25
      angle_max = 200

      -- Yaw angle psi is positive nose-left, whereas yaw demand is
      -- positive nose-right.  Hence we negate the yaw demand to
      -- accumulate the angle target.  
      target = cap $ target' + angle_max * (yaw demands) * dt

      error = target - (psi state)

      error' = [0] ++ error

      deriv = (error - error') / dt

      yaw' = kp * error + kd * deriv

      -- Return the result negated, so demand will still be nose-right
      -- positive
      demands' = Demands (thrust demands) (roll demands) (pitch demands) yaw'

      -- Reset target on zero thrust
      target' = [0] ++ (if (thrust demands == 0) then (psi state) else target)
