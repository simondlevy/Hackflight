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

kp_pitch_roll = 0.002 :: SFloat    
ki_pitch_roll = 0.003 :: SFloat    
kd_pitch_roll = 0.0005 :: SFloat   

kp_yaw = 0.003 :: SFloat           
ki_yaw = 0.0005 :: SFloat          
kd_yaw = 0.0000015 :: SFloat       

throttle_down = 1060 :: SInt32;

runPitchRollPidController dt reset demand angle dangle integral_prev 

  =  (error, integral, integral_prev) where

    error = demand - angle

    integral = integral_prev + error * dt

angleController dt throttle state demands = demands' where

  reset = false -- throttle < throttle_down

  -- roll_out = runRollPid dt reset (roll demands) (phi state) (dphi state)

  demands' = demands

