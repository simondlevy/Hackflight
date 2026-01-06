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

module YawAngleController where

import Language.Copilot
import Copilot.Compile.C99

import Num

kp = 6
ki = 1
kd = 0.35
ilimit = 360
demand_max = 200

dt :: SFloat
dt = extern "stream_dt" Nothing

state_psi :: SFloat
state_psi = extern "stream_psi" Nothing

demand_yaw :: SFloat
demand_yaw = extern "stream_yaw" Nothing

{-- 
  Demand is input as yawAngle target in meters and output as climb rate in
  meters per second.
--}

yawAngleController :: SFloat

yawAngleController = demand where

    target = cap $ target' + demand_max * demand_yaw * dt

    error = cap $ target - state_psi

    integral = constrainabs (integral' + error * dt) ilimit

    deriv = if dt > 0 then (error - error') / dt else 0

    demand = kp * error + ki * integral + kd * deriv

    target' = [0] ++ target 
    integral' = [0] ++ integral
    error' = [0] ++ error

cap :: SFloat -> SFloat

cap angle = if angle > 180 then angle - 360
            else if angle < (-180) then angle + 360
            else angle
