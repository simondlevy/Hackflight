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

kp_pitch_roll = 0.002 :: SFloat    
ki_pitch_roll = 0.003 :: SFloat    
kd_pitch_roll = 0.0005 :: SFloat   

kp_yaw = 0.003 :: SFloat           
ki_yaw = 0.0005 :: SFloat          
kd_yaw = 0.0000015 :: SFloat       

runPitchRoll dt reset demand angle dangle integral =  (output, integral') where

    error = demand - angle

    integral' = constrain
               (if reset then 0 else integral + error * dt)
               (-i_limit) i_limit

    output = kp_pitch_roll * error + ki_pitch_roll * integral' - kd_pitch_roll * dangle


runYaw dt reset demand dpsi' = yaw_PID where

  error = demand - dpsi'

  integral = constrain
               (if reset then 0 else integral' + error * dt)
               (-i_limit) i_limit

  derivative = (error - error') / dt

  yaw_PID = kp_yaw * error + ki_yaw * integral' - kd_yaw * derivative

  integral' = [0] ++ integral

  error' = [0] ++ error

angleController dt reset rollDemand pitchDemand yawDemand phi' theta' dphi' dtheta' dpsi' =
  (rollDemand', pitchDemand', yawDemand') where

  (rollDemand', roll_integral) = runPitchRoll dt reset rollDemand phi' dphi' roll_integral'

  roll_integral' = [0] ++ roll_integral

  (pitchDemand', pitch_integral) = runPitchRoll dt reset pitchDemand theta' dtheta' pitch_integral'

  yawDemand' = runYaw dt reset yawDemand dpsi'

  pitch_integral' = [0] ++ pitch_integral
