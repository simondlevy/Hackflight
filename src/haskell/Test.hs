{--
  Haskell Copilot support for Hackflight test sketch

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Test where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((++), (==), (&&), not, (/))

-- Core
import Hackflight
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

motorfun :: SWord8 -> SWord8 -> SWord8 -> SFloat
motorfun index target percent = if index == target then (unsafeCast percent) / 100 else 0

spec = do

  -- Make flags for startup, loop
  let flipflop = not flipflop' where flipflop' = [False] ++ flipflop
  let running = if not flipflop then true else running' where running' = [False] ++ running
  let starting = not running

  -- Run the Hackflight algorithm
  let (vstate, armed, motors, led) = hackflight receiver sensors pidfuns

  -- Run the serial comms parser
  let (msgtype, sending, payindex, checked) = parse serialAvailable serialByte

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startLed" starting [arg led_pin]

  -- Do some other stuff in loop
  trigger "stream_updateTime" running []
  trigger "stream_updateImu" running []
  trigger "stream_updateReceiver" running []
  trigger "stream_writeLed" running [arg led_pin, arg led]

  trigger "stream_serialUpdate" running []
  trigger "stream_serialRead" serialAvailable []

  trigger "stream_handleSerialInput" sending [  arg msgtype
                                              , arg $ phi vstate
                                              , arg $ theta vstate
                                              , arg $ psi vstate
                                             ]

  trigger "stream_updateSerialOutput" true []

  let motor_index = if msgtype == 215 && payindex == 1 then serialByte
                    else motor_index' where motor_index' = [0] ++ motor_index

  let motor_percent = if msgtype == 215 && payindex == 2 then serialByte
                      else motor_percent' where motor_percent' = [0] ++ motor_percent

  let m1_val = motorfun motor_index 1 motor_percent
  let m2_val = motorfun motor_index 2 motor_percent
  let m3_val = motorfun motor_index 3 motor_percent
  let m4_val = motorfun motor_index 4 motor_percent

  trigger "stream_debug" running [arg m1_val, arg m2_val, arg m3_val, arg m4_val]
  

-- Compile the spec
main = reify spec >>= compile "copilot"
