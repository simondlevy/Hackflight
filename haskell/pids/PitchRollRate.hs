{--
  Pitch/roll angular rate PID-control algorithm for real and simulated flight
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

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

run reset kp kd dt target actual prev = (demand, prev') where

    error = target - actual

    demand = kp * error + kd * (error - prev) / dt

    prev' = if reset then 0 else error

{--
  Demands are input as angular velocities in degrees per second and
  output in uints appropriate for our motors.
--}

pitchRollRateController dt state demands = demands' where

  kp = 1.25e-2
  kd = 1.25e-4

  nothrust = (thrust demands) == 0

  (rollDemand, rollPrev) = 
    run nothrust kp kd dt (roll demands) (dphi state) rollPrev'

  rollPrev' = [0] ++ rollPrev

  (pitchDemand, pitchPrev) = 
    run nothrust kp kd dt (pitch demands) (dtheta state) pitchPrev'

  pitchPrev' = [0] ++ pitchPrev

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
