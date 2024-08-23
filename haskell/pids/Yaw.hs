{--
  Yaw PID-control algorithm for real and simulated flight
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

module Yaw where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils


i_limit = 25.0 :: SFloat     

kp = 0.003 :: SFloat           
ki = 0.0005 :: SFloat          
kd = 0.0000015 :: SFloat       

yawController dt reset demand gyroZ = yaw_PID where

  error = demand - gyroZ

  integral = constrain
               (if reset then 0 else integral' + error * dt)
               (-i_limit) i_limit

  derivative = (error - error') / dt

  yaw_PID = kp * error + ki * integral' - kd * derivative

  integral' = [0] ++ integral

  error' = [0] ++ error
