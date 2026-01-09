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

module YawRateController where

import Language.Copilot
import Copilot.Compile.C99

import Num

kp = 120
ki = 16.7
ilimit = 166.7

dt :: SFloat
dt = extern "stream_dt" Nothing

state_dpsi :: SFloat
state_dpsi = extern "stream_dpsi" Nothing

{-- 
    Input is angular rate demand (deg/sec) and actual angular
    rate from gyro; ouputput is arbitrary units scaled for motors.
--}

yawRateController :: SFloat -> SFloat

yawRateController demand = demand' where

    error = demand - state_dpsi

    integral = constrainabs (integral' + error * dt) ilimit

    demand' = kp * error + ki * integral
 
    integral' = [0] ++ integral
