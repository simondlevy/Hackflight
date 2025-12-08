{--
  Copyright (C) 2025 Simon D. Levy
 
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

module ClimbRateController where

import Language.Copilot
import Copilot.Compile.C99

import Num

-- PID constants
kp = 25
ki = 15
ilimit = 5000

-- Appropriate thrust values for our DIY quadcopter
thrust_base  = 36000
thrust_min   = 20000
thrust_max   = 65535
thrust_scale = 1000

hovering :: SBool
hovering = extern "stream_controlled" Nothing

dt :: SFloat
dt = extern "stream_dt" Nothing

state_z :: SFloat
state_z = extern "stream_z" Nothing

state_dz :: SFloat
state_dz = extern "stream_dz" Nothing

landing_altitude_m :: SFloat
landing_altitude_m = extern "stream_landing_altitude_m" Nothing

{-- 

  Demand is input as climb rate in meters per second and output as arbitrary
  positive value to be scaled according to motor characteristics.

--}

climbRateController ::  SFloat -> SFloat

climbRateController climbrate = spin where

    airborne = hovering || (state_z > landing_altitude_m)

    error = climbrate - state_dz

    integral = if airborne
               then constrainabs (integral' + error * dt) ilimit
               else 0

    integral' = [0] ++ integral

    thrust = kp * error + ki * integral

    spin = if airborne 
           then constrain (thrust * thrust_scale + thrust_base) 
                          thrust_min thrust_max
           else 0
