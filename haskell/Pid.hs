{--
  PID control support for real and simulated flight controllers
 
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

module Pid where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

type PidController = SBool -> SFloat -> State -> Demands -> Demands

 
pidController kp ki kd dt ilimit target actual capfun error' integ' = 
  (output, error, integ) where

  error = capfun $ target - actual

  integ = constrain (integ' + error * dt) (-ilimit) ilimit

  deriv = (error - error') / dt

  output = kp * error + ki * integ + kd * deriv


piController kp ki dt ilimit target actual integ' = (output, integ) where

  error = target - actual

  integ = constrain (integ' + error * dt) (-ilimit) ilimit

  output = kp * error + ki * integ
