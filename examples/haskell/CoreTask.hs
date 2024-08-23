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
import Demands
import Mixers
import Sensors
import State
import Utils

-- PID controllers
import Angle

-- Constants

clock_rate = RATE_2000_HZ

pitch_roll_demand_post_scale = 30 :: SFloat -- deg

yaw_demand_pre_scale = 160 :: SFloat -- deg/sec

-- Streams from C++ ----------------------------------------------------------

throttle_stick :: SFloat
throttle_stick = extern "stream_throttle" Nothing

roll_stick :: SFloat
roll_stick = extern "stream_roll" Nothing

pitch_stick :: SFloat
pitch_stick = extern "stream_pitch" Nothing

yaw_stick :: SFloat
yaw_stick = extern "stream_yaw" Nothing

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

  state = State  0 
                 0 
                 0 
                 0 
                 state_phi 
                 state_dphi 
                 state_theta 
                 state_dtheta 
                 state_psi 
                 state_dpsi

  stickDemands = Demands throttle_stick 
                 roll_stick 
                 pitch_stick 
                 (yaw_demand_pre_scale * yaw_stick)

  dt = rateToPeriod clock_rate

  pids = [ angleController dt ]

  demands' = foldl (\demand pid -> pid state demand) stickDemands pids

  motors = runBetaFlightQuadX $ Demands (thrust demands')
                                        (pitch_roll_demand_post_scale * (roll demands'))
                                        (pitch_roll_demand_post_scale * (pitch demands'))
                                        (yaw demands')

------------------------------------------------------------------------------
 
spec = do

    let motors = step

    let (m1, m2, m3, m4) = motors

    trigger "setMotors" true [arg $ m1, arg $ m2, arg $ m3, arg $ m4] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
