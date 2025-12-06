{--
  Copyright (C) 2025 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module AltitudeController where

import Language.Copilot
import Copilot.Compile.C99

import Num

kp = 2
ki = 0.5
ilimit = 5000
vel_max = 1
vel_max_overhead = 1.10
landing_speed_mps = 0.15

dt :: SFloat
dt = extern "stream_dt" Nothing

demand_thrust :: SFloat
demand_thrust = extern "stream_thrust" Nothing

state_z :: SFloat
state_z = extern "stream_z" Nothing

airborne :: SBool
airborne = extern "stream_airborne" Nothing

{-- 
  Demand is input as altitude target in meters and output as climb rate in meters
  per second.
--}

altitudeController :: SFloat

altitudeController = climbrate where

    error = demand_thrust - state_z

    integral = if airborne 
               then constrainabs (integral' + error * dt) ilimit
               else 0

    integral' = [0] ++ integral

    climbrate = if airborne
                then constrainabs (kp * error + ki * integral)
                                  ((max vel_max 0.5) * vel_max_overhead)
                else (-landing_speed_mps)
