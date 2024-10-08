{--
  Pitch/roll/yaw PID-control algorithm for real and simulated flight
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

module Angle where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

i_limit = 25.0 :: SFloat     

throttle_down = 0.06 :: SFloat

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

angleController dt state demands = demands' where

  throttle_demand = thrust demands

  reset = throttle_demand < throttle_down

  (roll_demand', roll_integral) =
    runPitchRoll dt reset (roll demands) (phi state) (dphi state) roll_integral'

  roll_integral' = [0] ++ roll_integral

  (pitch_demand', pitch_integral) =
    runPitchRoll dt reset (pitch demands) (theta state) (dtheta state) pitch_integral'

  yaw_demand' = runYaw dt reset (yaw demands) (dpsi state)

  pitch_integral' = [0] ++ pitch_integral

  demands' = Demands throttle_demand roll_demand' pitch_demand' yaw_demand'
