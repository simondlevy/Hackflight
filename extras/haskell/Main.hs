{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

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

import Hackflight

import State

import Time

spec = do

{--
  let receiver = makeReceiver 4.0

  -- These sensors will be run right-to-left via composition
  let sensors = [gyrometer, quaternion]

  let ratePid = rateController 0.225    -- Kp
                               0.001875 -- Ki
                               0.375    -- Kd
                               0.4      -- windupMax
                               40       -- maxDegreesPerSecond

  let yawPid = yawController 2.0 -- Kp
                             0.1 -- Ki
                             0.4 -- windupMax

  let levelPid = levelController 0.2 -- Kp
                                 45  -- maxAngleDegrees

  -- Pos-hold goes first so that it can access roll/pitch demands from receiver
  let pidControllers = [ratePid, yawPid, levelPid]

  let mixer = quadXAPMixer

  let (motors, ledState) = hackflight receiver sensors pidControllers mixer

  -- Send the motor values to the external C function
  trigger "copilot_runMotors" true [arg $ m1 motors,
                                    arg $ m2 motors,
                                    arg $ m3 motors,
                                    arg $ m4 motors]

--}

  -- Send the LED state to the external C function
  trigger "copilot_setLed" true [arg $ time < 5]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
