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
import Altitude
import ClimbRate
import PitchRollAngle
import PitchRollRate
import Position
import YawAngle
import YawRate

-- Constants

status_landed =     0 :: SInt8
status_taking_off = 1 :: SInt8
status_flying =     2 :: SInt8

altitude_target_initial = 0.2 :: SFloat
throttle_zero           = 0.05 :: SFloat
throttle_scale          = 0.005 :: SFloat
zground                 = 0.05 :: SFloat

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "stream_stickDemands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "stream_vehicleState" Nothing

step = motors where

  state = liftState stateStruct

  stickDemands = liftDemands demandsStruct

  altitude_target = if status == status_flying 
                    then altitude_target' + throttle_scale * (thrust stickDemands)
                    else if status' == status_landed
                    then altitude_target_initial
                    else altitude_target'

  altitude_target' = [0] ++ altitude_target

  status = if status' == status_taking_off && (zz state) > zground
           then status_flying
           else if status' == status_flying && (zz state) <= zground
           then status_landed
           else if status' == status_landed && (thrust stickDemands) > throttle_zero
           then status_taking_off
           else status'

  status' = [0] ++ status

  landed = status == status_landed

  dt = rateToPeriod clock_rate

  pids = [positionController dt,
          pitchRollAngleController dt,
          pitchRollRateController dt,
          altitudeController altitude_target dt,
          climbRateController (not landed) dt,
          yawAngleController dt,
          yawRateController dt]

  demands = foldl (
     \demand pid -> pid state demand) stickDemands pids

  motors = runCF $ Demands (thrust demands)
                           (roll demands) 
                           (pitch demands)
                           (yaw demands)

------------------------------------------------------------------------------
 
spec = do

    let motors = step

    let (m1, m2, m3, m4) = motors

    trigger "setMotors" true [arg $ m1, arg $ m2, arg $ m3, arg $ m4] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
