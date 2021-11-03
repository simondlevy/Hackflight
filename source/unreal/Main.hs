{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import Hackflight
import State
import Demands
import Receiver
import Time
import Mixer

-- Sensors
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

-- sensors = [gyrometer, quaternion, altimeter, opticalFlow]
sensors = [gyrometer, quaternion, altimeter, opticalFlow]

-- PID controllers are applied last-to-first.  Pos-hold is last in list
-- so that it will can access to the unmodifed receiver demands.
pidfuns = [  
             altHoldController 0.75 1.5   -- Kp, Ki
           , levelController 0.2 -- Kp
           , yawController 1.0625 0.005625 -- Kp, Ki
           , rateController 0.225  0.001875 0.375 -- Kp, Ki, Kd 
           , posHoldController 0.1  -- Kp
          ]

------------------------------------------------------------

spec = do

  -- Run the main Hackflight algorithm, getting the motor spins
  let (rdemands, pdemands, motors) = hackflightSim receiver sensors pidfuns

  -- Send the motor values using the external C function
  trigger "stream_writeMotors" true [  arg $ m1 motors
                                     , arg $ m2 motors
                                     , arg $ m3 motors
                                     , arg $ m4 motors ]

  trigger "stream_debugReceiver" true [  arg $ throttle rdemands
                                       , arg $ roll rdemands
                                       , arg $ pitch rdemands
                                       , arg $ yaw rdemands ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
