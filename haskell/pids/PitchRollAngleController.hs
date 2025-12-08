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

module PitchRollAngleController where

import Language.Copilot
import Copilot.Compile.C99

import Num

kp = 6
ki = 3
ilimit = 20

dt :: SFloat
dt = extern "stream_dt" Nothing

state_phi :: SFloat
state_phi = extern "stream_phi" Nothing

state_theta :: SFloat
state_theta = extern "stream_theta" Nothing

{-- 
  Demand is input as angles in degrees and output as angular
  velocities in degrees per second:
  
  roll: right-down positive
  
  pitch: nose-down positive
  --}

pitchRollAngleController :: (SFloat, SFloat) -> (SFloat, SFloat)

pitchRollAngleController (roll, pitch) = (roll', pitch') where

    -- Run PIDs on body-coordinate velocities
    (roll',  roll_integral) = runAxis dt roll  state_phi roll_integral'

    (pitch', pitch_integral) = runAxis dt pitch state_theta pitch_integral'

    roll_integral' = [0] ++ roll_integral
    pitch_integral' = [0] ++ pitch_integral

    runAxis dt demand measured integral = (demand', integral') where

        error = demand - measured

        integral' = constrainabs (integral + error * dt) ilimit

        demand' = kp * error + ki * integral
