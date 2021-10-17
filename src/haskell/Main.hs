{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((<), (>), (++), not)

-- Core
import HackflightFull
import Receiver
import Demands
import State

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid(rateController)
import YawPid(yawController)
import LevelPid(levelController)

-- Misc
import Utils

m1_pin = 13 :: SWord8 
m2_pin = 16 :: SWord8 
m3_pin = 3  :: SWord8 
m4_pin = 11 :: SWord8 

led_pin = 18 :: SWord8 

receiver = makeReceiverWithTrim (AxisTrim 0.0 0.05 0.045) 4.0

-- PID controllers are applied last-to-first
pidfuns = [  yawController 1.0625 0.005625 -- Kp, Ki
           , rateController 0.225  0.001875 0.375 -- Kp, Ki, Kd 
           , levelController 0.2 -- Kp
          ]

sensors = [gyrometer, quaternion]

spec = do

  -- Make flags for startup, loop
  let count = (z::Stream Word64) + 1 where z = [0] ++ count
  let running = count > 1
  let starting = not running

  let  (state, rdemands, pdemands, led) = hackflight receiver sensors pidfuns

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

  trigger "stream_runHackflight" running [  
                                            arg $ receiverAux1 > 0
                                          , arg $ (throttle rdemands) < (-0.995)
                                          , arg $ phi state
                                          , arg $ theta state
                                          , arg $ psi state
                                          , arg $ throttle pdemands
                                          , arg $ roll pdemands
                                          , arg $ pitch pdemands
                                          , arg $ yaw pdemands 
                                         ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
