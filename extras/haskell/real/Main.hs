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
import Time
import Demands
import Receiver
import Mixer

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid
import YawPid
import LevelPid

-- Serial comms
import Serial

spec = do

  let receiver = makeReceiver 4.0

  -- These sensors will be run right-to-left via composition
  let sensors = [gyrometer, quaternion]

  let ratePid = rateController 0.225    -- Kp
                               0.001875 -- Ki
                               0.375    -- Kd
                               0.4      -- windupMax
                               40       -- maxDegreesPerSecond

  -- Set up some PID controllers

  let yawPid = yawController 2.0 -- Kp
                             0.1 -- Ki
                             0.4 -- windupMax

  let levelPid = levelController 0.2 -- Kp
                                 45  -- maxAngleDegrees

  -- Pos-hold goes first so that it can access roll/pitch demands from receiver
  let pidControllers = [ratePid, yawPid, levelPid]

  let mixer = quadXAPMixer

  -- Run the main Hackflight algorithm, getting the motor spins and LED state
  let (motors, led, byte) = hackflight receiver sensors pidControllers mixer Serial

  -- Send the motor values using the external C function
  trigger "copilot_runMotors" true [arg $ m1 motors,
                                    arg $ m2 motors,
                                    arg $ m3 motors,
                                    arg $ m4 motors]

  -- trigger "copilot_debug" true [arg foo]

  -- Send the LED using external C function
  trigger "copilot_setLed" true [arg led]

  -- Send and retrieve serial comms
  trigger "copilot_sendSerial" true [arg byte]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
