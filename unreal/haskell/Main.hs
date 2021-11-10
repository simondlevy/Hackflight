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

-- Sensors
import Sensor
import Gyrometer
import Quaternion
import Altimeter
import OpticalFlow

-- PID controllers
import PidController
import RatePid(rateController)
import YawPid(yawController)
import LevelPid(levelController)
import AltHoldPid(altHoldController)
import PosHoldPid(posHoldController)

------------------------------------------------------------

receiver = makeReceiver 4.0

sensors = [gyrometer, quaternion, altimeter, opticalFlow]

pidfuns = [  
             altHoldController 0.75 1.5   -- Kp, Ki
           , levelController 0.2 -- Kp
           , yawController 1.0625 0.005625 -- Kp, Ki
           , rateController 0.225  0.001875 0.375 -- Kp, Ki, Kd 
           , posHoldController 0.1  -- Kp
          ]

------------------------------------------------------------

hackflight :: Receiver -> [Sensor] -> [PidFun] -> StateFun -> Mixer -> SafetyFun
  -> (Demands, State, Demands, Motors)

hackflight receiver sensors pidfuns statefun mixer safefun
  = (rdemands, vstate, pdemands, motors)

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the current state
    vstate = compose sensors (statefun vstate)

    -- Periodically get the demands by composing the PID controllers over the receiver
    -- demands
    (_, _, pdemands) = compose pidfuns (vstate, timerReady 300, rdemands)

    -- Run mixer on demands to get motor values
    motors = (mixerfun mixer) safefun pdemands


spec = do

  let motorfun = \m -> constrain m

  let statefun = \_ -> State 0 0 0 0 0 0 0 0 0 0 0 0

  let (_, _, _, motors) = hackflight receiver sensors pidfuns statefun quadxap motorfun

  -- Call some C routines for open-loop control and sensing
  trigger "stream_getReceiverDemands" true []
  trigger "stream_getGyrometer" true []
  trigger "stream_getQuaternion" true []
  trigger "stream_getOpticalFlow" true []

  -- Send the motor values using the external C function
  trigger "stream_writeMotors" true [  arg $ m1 motors
                                     , arg $ m2 motors
                                     , arg $ m3 motors
                                     , arg $ m4 motors ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
