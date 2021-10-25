{--
  Haskell Copilot support for Hackflight test sketch

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Test where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((<), (>), (++), not)

-- Core
import HackflightFull
import Receiver
import Demands
import State
import Mixer

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid(rateController)
import YawPid(yawController)
import LevelPid(levelController)

-- Serial comms
import Serial
import Parser

-- Misc
import Utils

led_pin = 13 :: SWord8 

receiver = makeReceiver 4.0

sensors = [gyrometer, quaternion]

-- PID controllers are applied last-to-first
pidfuns = [  yawController 1.0625 0.005625 -- Kp, Ki
           , rateController 0.225  0.001875 0.375 -- Kp, Ki, Kd 
           , levelController 0.2 -- Kp
          ]

-- mixer = quadXMWMixer

spec = do

  -- Make flags for startup, loop
  let flipflop = not flipflop' where flipflop' = [False] ++ flipflop
  let running = if not flipflop then true else running' where running' = [False] ++ running
  let starting = not running

  -- Run the Hackflight algorithm
  let (vstate, armed, motors, led, pstate) = hackflight receiver
                                                        sensors
                                                        pidfuns
                                                        serialAvailable
                                                        serialByteIn
  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startLed" starting [arg led_pin]

  -- Do some other stuff in loop
  trigger "stream_updateTime" running []
  trigger "stream_updateImu" running []
  trigger "stream_updateReceiver" running []
  trigger "stream_writeLed" running [arg led_pin, arg led]

  trigger "stream_run" running [  arg $ phi vstate
                                , arg $ theta vstate
                                , arg $ psi vstate
                                , arg armed ]

-- Compile the spec
main = reify spec >>= compile "copilot"
