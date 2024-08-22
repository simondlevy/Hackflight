{--
  LambdaFlight core algorithm: reads open-loop demands and
  state as streams; runs PID controllers and motor mixers
 
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

module CoreTask where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Constants
import Demands
import Mixers
import Sensors
import State
import Utils

import Constants

-- PID controllers
import Angle
import ClimbRate
import Position

import PitchRollAngle
import PitchRollRate
import YawAngle
import YawRate

-- Constants

k_climbrate = 25 :: SFloat

thrust_takeoff = 56 :: SFloat

thrust_base = 55.385 :: SFloat

-- Streams from C++ ----------------------------------------------------------

throttle_stick :: SFloat
throttle_stick = extern "stream_throttle" Nothing

roll_stick :: SFloat
roll_stick = extern "stream_roll" Nothing

pitch_stick :: SFloat
pitch_stick = extern "stream_pitch" Nothing

yaw_stick :: SFloat
yaw_stick = extern "stream_yaw" Nothing

hitTakeoffButton :: SBool
hitTakeoffButton = extern "stream_hitTakeoffButton" Nothing

completedTakeoff :: SBool
completedTakeoff = extern "stream_completedTakeoff" Nothing

state_dx :: SFloat
state_dx = extern "stream_dx" Nothing

state_dy :: SFloat
state_dy = extern "stream_dy" Nothing

state_z :: SFloat
state_z = extern "stream_z" Nothing

state_dz :: SFloat
state_dz = extern "stream_dz" Nothing

state_phi :: SFloat
state_phi = extern "stream_phi" Nothing

state_dphi :: SFloat
state_dphi = extern "stream_dphi" Nothing

state_theta :: SFloat
state_theta = extern "stream_theta" Nothing

state_dtheta :: SFloat
state_dtheta = extern "stream_dtheta" Nothing

state_psi :: SFloat
state_psi = extern "stream_psi" Nothing

state_dpsi :: SFloat
state_dpsi = extern "stream_dpsi" Nothing

step = motors where

  state = State  state_dx 
                 state_dy 
                 state_z 
                 state_dz 
                 state_phi 
                 state_dphi 
                 state_theta 
                 state_dtheta 
                 state_psi 
                 state_dpsi

  stickDemands = Demands throttle_stick roll_stick pitch_stick yaw_stick

  dt = rateToPeriod clock_rate

  pids = [climbRateController hitTakeoffButton completedTakeoff,
          positionController dt,
          angleController dt,
          pitchRollAngleController dt,
          pitchRollRateController dt,
          yawAngleController dt,
          yawRateController dt]

  demands' = foldl (\demand pid -> pid state demand) stickDemands pids

  motors = runCF $ Demands (thrust demands')
                           (roll demands') 
                           (pitch demands')
                           (yaw demands')

------------------------------------------------------------------------------
 
spec = do

    let motors = step

    let (m1, m2, m3, m4) = motors

    trigger "setMotors" true [arg $ m1, arg $ m2, arg $ m3, arg $ m4] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
