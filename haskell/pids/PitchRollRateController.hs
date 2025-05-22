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

module PitchRollRateController where

import Language.Copilot
import Copilot.Compile.C99

import Num

kp = 125
ki = 250
kd = 1.25
ilimit = 33

dt :: SFloat
dt = extern "stream_dt" Nothing

state_dphi :: SFloat
state_dphi = extern "stream_dphi" Nothing

state_dtheta :: SFloat
state_dtheta = extern "stream_dtheta" Nothing

{-- 
  Demands are input as angular velocities in degrees per second and
  output as as arbitrary values to be scaled according to motor
  characteristics:
  
  roll:  input roll-right positive => output positive
  
  pitch: input nose-down positive => output positive
--}

pitchRollRateController :: SBool -> (SFloat, SFloat) -> (SFloat, SFloat)

pitchRollRateController airborne (roll, pitch) = (roll', pitch') where

    (roll',  roll_integral, roll_error) =
       runAxis airborne dt roll state_dphi roll_integral'  roll_error'

    (pitch', pitch_integral, pitch_error) =
      runAxis airborne dt pitch state_dtheta pitch_integral' pitch_error'

    roll_integral' = [0] ++ roll_integral
    pitch_integral' = [0] ++ pitch_integral

    roll_error' = [0] ++ roll_error
    pitch_error' = [0] ++ pitch_error

    runAxis airborne dt demand measured integral error' =
      (demand', integral', error'') where

        error = demand - measured

        deriv = if dt > 0 then (error - error') / dt else 0

        demand' = kp * error + ki * integral + kd * deriv

        integral' = if airborne
                    then constrainabs (integral + error * dt) ilimit
                    else 0

        error'' = if airborne then error else 0
