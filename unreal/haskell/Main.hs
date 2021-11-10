{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import State
import Demands
import Receiver
import Time
import Mixers
import Motors
import Dynamics
import Utils

-- PID controllers
import PidController
import RatePid(rateController)
import YawPid(yawController)
import LevelPid(levelController)
import AltHoldPid(altHoldController)
import PosHoldPid(posHoldController)

------------------------------------------------------------

receiver = makeReceiver 4.0 -- demand scale

pidfuns = [  
             altHoldController 0.75 1.5   -- Kp, Ki
           , levelController 0.2 -- Kp
           , yawController 1.0625 0.005625 -- Kp, Ki
           , rateController 0.225  0.001875 0.375 -- Kp, Ki, Kd 
           , posHoldController 0.1  -- Kp
          ]

------------------------------------------------------------

spec = do

  -- Get receiver demands from external C functions
  let rdemands = getDemands receiver

  -- Get vehicle state directly from simulation dynamics
  let state = State 0 -- X
                    stream_stateDx
                    0 -- Y
                    stream_stateDy
                    0 -- Z
                    stream_stateDz
                    stream_statePhi
                    stream_stateDphi
                    stream_stateTheta
                    stream_stateDtheta
                    stream_statePsi
                    stream_stateDpsi

  -- Periodically get the demands by composing the PID controllers over the receiver
  -- demands
  let (_, _, pdemands) = compose pidfuns (state, timerReady 300, rdemands)

  -- Run mixer on demands to get motor values
  let motors = (mixerfun quadxap) constrain pdemands

  -- Call some C routines for getting receiver demands
  trigger "stream_getReceiverDemands" true []

  -- Send the motor values using the external C function
  trigger "stream_writeMotors" true [  arg $ m1 motors
                                     , arg $ m2 motors
                                     , arg $ m3 motors
                                     , arg $ m4 motors ]

-- Compile the spec
main = reify spec >>= compile "hackflight"

stream_stateDx :: SFloat
stream_stateDx = extern "stream_stateDx" Nothing

stream_stateDy :: SFloat
stream_stateDy = extern "stream_stateDy" Nothing

stream_stateDz :: SFloat
stream_stateDz = extern "stream_stateDz" Nothing

stream_statePhi :: SFloat
stream_statePhi = extern "stream_statePhi" Nothing

stream_stateDphi :: SFloat
stream_stateDphi = extern "stream_stateDphi" Nothing

stream_stateTheta :: SFloat
stream_stateTheta = extern "stream_stateTheta" Nothing

stream_stateDtheta :: SFloat
stream_stateDtheta = extern "stream_stateDtheta" Nothing

stream_statePsi :: SFloat
stream_statePsi = extern "stream_statePsi" Nothing

stream_stateDpsi :: SFloat
stream_stateDpsi = extern "stream_stateDpsi" Nothing
