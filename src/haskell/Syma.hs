{--
  Haskell Copilot support for Hackflight on Syma quadcopter

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Syma where

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

m1_pin = 13 :: SWord8 
m2_pin = 16 :: SWord8 
m3_pin = 3  :: SWord8 
m4_pin = 11 :: SWord8 

led_pin = 18 :: SWord8 

receiver = makeReceiverWithTrim (AxisTrim 0.0 0.05 0.045) 4.0

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

  let (state, armed, motors, led, pstate) = hackflight receiver sensors pidfuns serialAvailable serialByteIn

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startI2C" starting []
  trigger "stream_startReceiver" starting []
  trigger "stream_startImu" starting []
  trigger "stream_startBrushedMotors" starting [arg m1_pin, arg m2_pin, arg m3_pin, arg m4_pin]
  trigger "stream_startLed" starting [arg led_pin]

  -- Do some other stuff in loop
  trigger "stream_updateImu" running []
  trigger "stream_updateReceiver" running []
  trigger "stream_updateTime" running []
  trigger "stream_writeLed" running [arg led_pin, arg led]

  trigger "stream_run" running [  arg $ phi state
                                , arg $ theta state
                                , arg $ psi state
                                , arg armed
                                , arg m1_pin
                                , arg m2_pin
                                , arg m3_pin
                                , arg m4_pin
                                , arg $ m1 motors
                                , arg $ m2 motors
                                , arg $ m3 motors
                                , arg $ m4 motors ]

-- Compile the spec
main = reify spec >>= compile "copilot"
