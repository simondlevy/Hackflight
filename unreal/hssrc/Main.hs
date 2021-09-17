{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import HackflightSim
import Safety
import Serial
import State
import Time
import Demands
import Receiver
import Mixer

-- Sensors
import Gyrometer
import Quaternion
import Altimeter
import OpticalFlow

-- PID controllers
import RatePid
import YawPid
import LevelPid
import AltHoldPid
import PosHoldPid

-- Serial comms (placeholder)

spec = do

  let receiver = makeReceiver 4.0

  -- These sensors will be run right-to-left via composition
  let sensors = [gyrometer, quaternion, altimeter, opticalFlow]

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

  let altHoldPid = altHoldController 0.75 -- Kp
                                     1.5  -- Ki
                                     0.4  -- windumpMax
                                     2.5  -- pilotVelZMax
                                     0.2  -- stickDeadband

  let posHoldPid = posHoldController 0.1 -- Kp
                                     0.2 -- stickDeadband

  -- Pos-hold goes first so that it can access roll/pitch demands from receiver
  let pidControllers = [posHoldPid, altHoldPid, ratePid, yawPid, levelPid]

  let mixer = quadXAPMixer

  -- Run the main Hackflight algorithm, getting the motor spins
  let motors = hackflightSim receiver sensors pidControllers mixer

  -- Send the motor values using the external C function
  trigger "copilot_writeMotors" true [arg $ value $ m1 motors,
                                      arg $ value $ m2 motors,
                                      arg $ value $ m3 motors,
                                      arg $ value $ m4 motors]

-- Compile the spec
main = reify spec >>= compile "copilot"
