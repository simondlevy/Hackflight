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

module PositionController where

import Language.Copilot
import Copilot.Compile.C99

import Num

kp = 25
ki = 1
ilimit = 5000
limit = 20
limit_overhead = 1.10

dt :: SFloat
dt = extern "stream_dt" Nothing

demand_roll :: SFloat
demand_roll = extern "stream_roll" Nothing

demand_pitch :: SFloat
demand_pitch = extern "stream_pitch" Nothing

state_dx :: SFloat
state_dx = extern "stream_dx" Nothing

state_dy :: SFloat
state_dy = extern "stream_dy" Nothing

state_psi :: SFloat
state_psi = extern "stream_psi" Nothing


{-- 
  Demands are input as normalized interval [-1,+1] and output as
  angles in degrees.
 
  roll:  input left positive => output negative
 
  pitch: input forward positive => output negative
 --}

positionController :: (SFloat, SFloat)

positionController = (roll, pitch) where

    psi = deg2rad state_psi

    -- Rotate world-coordinate velocities into body coordinates
    cospsi = cos psi
    sinpsi = sin psi
    dxb =  state_dx * cospsi + state_dy * sinpsi
    dyb = -state_dx * sinpsi + state_dy * cospsi       

    -- Run PIDs on body-coordinate velocities
    (roll,  integralY) = runAxis dt demand_roll  dyb integralY'
    (pitch, integralX) = runAxis dt demand_pitch dxb integralX'

    integralX' = [0] ++ integralX
    integralY' = [0] ++ integralY

    runAxis dt demand measured integral = (demand', integral') where

        error = demand - measured

        integral' = constrainabs (integral + error * dt) ilimit

        demand' = constrainabs (kp * error + ki * integral) limit
