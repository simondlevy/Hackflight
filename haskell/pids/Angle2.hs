{--
  Pitch/roll PID-control algorithm for real and simulated flight
  controllers
 
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

module Angle2 where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils


i_limit = 25.0 :: SFloat     

kp = 0.002 :: SFloat    
ki = 0.003 :: SFloat    
kd = 0.0005 :: SFloat   

runPitchRoll dt reset demand angle dangle integral =  (output, integral') where

    error = demand - angle

    integral' = constrain
               (if reset then 0 else integral + error * dt)
               (-i_limit) i_limit

    output = kp * error + ki * integral' - kd * dangle


angleController dt reset rollDemand pitchDemand phi' theta' dphi' dtheta' =
  (rollDemand', pitchDemand') where

  (rollDemand', roll_integral) = runPitchRoll dt reset rollDemand phi' dphi' roll_integral'

  roll_integral' = [0] ++ roll_integral

  (pitchDemand', pitch_integral) = runPitchRoll dt reset pitchDemand theta' dtheta' pitch_integral'

  pitch_integral' = [0] ++ pitch_integral
