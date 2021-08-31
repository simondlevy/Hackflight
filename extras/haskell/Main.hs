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

spec = do

  let receiver = makeReceiver (ChannelMap chan1 chan2 chan3 chan4 chan7 chan5) 4.0

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

  let mixer = QuadXAPMixer

  let (demands, state) = hackflight receiver sensors pidControllers

  trigger "copilot_debug" true [arg $ phi state]

  -- Use the mixer to convert the demands into motor values
  let (m1, m2, m3, m4) = getMotors mixer demands

  -- Send the motor values to the external C function
  trigger "copilot_runMotors" true [arg m1, arg m2, arg m3, arg m4]

-- Compile the spec
main = reify spec >>= compile "copilot"
