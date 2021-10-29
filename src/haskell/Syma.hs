{--
  Haskell Copilot support for Hackflight on Syma quadcopter

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Syma where

import Language.Copilot hiding(xor)
import Copilot.Compile.C99
import Prelude hiding((++), (==), (&&), (/), (*), (+), not, xor)

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

motorfun :: SBool -> SFloat -> SWord8 -> SWord8 -> SWord8 -> SFloat
motorfun armed flying_value index target percent =
  if armed then flying_value
  else if index == target then (unsafeCast percent) / 100
  else 0

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

  trigger "stream_serialUpdate" running []
  trigger "stream_serialRead" serialAvailable []

  let paysize = if msgtype == 121 then 6 else if msgtype == 122 then 3 else 0 :: SWord8

  let outsize = 4 * paysize

  let val00 = if msgtype == 121 then receiverThrottle
              else if msgtype == 122 then (phi vstate)
              else 0

  let val01 = if msgtype == 121 then receiverRoll
              else if msgtype == 122 then (theta vstate)
              else 0

  let val02 = if msgtype == 121 then receiverPitch
              else if msgtype == 122 then (psi vstate)
              else 0

  let val03 = if msgtype == 121 then receiverYaw else 0
  let val04 = if msgtype == 121 then receiverAux1 else 0
  let val05 = if msgtype == 121 then receiverAux2 else 0

  trigger "stream_serialSend" sending [ arg (0x24::SWord8)        -- '$'
                                      , arg (0x4D::SWord8)        -- 'M'
                                      , arg (0x3E::SWord8)        -- '>'
                                      , arg outsize
                                      , arg msgtype
                                      , arg $ xor outsize msgtype -- CRC
                                      , arg paysize
                                      , arg $ val00
                                      , arg $ val01
                                      , arg $ val02
                                      , arg $ val03
                                      , arg $ val04
                                      , arg $ val05
                                      ]

  let motor_index = if msgtype == 215 && payindex == 1 then serialByte
                    else motor_index' where motor_index' = [0] ++ motor_index

  let motor_percent = if msgtype == 215 && payindex == 2 then serialByte
                      else motor_percent' where motor_percent' = [0] ++ motor_percent

  let m1_val = motorfun armed (m1 motors) motor_index 1 motor_percent
  let m2_val = motorfun armed (m2 motors) motor_index 2 motor_percent
  let m3_val = motorfun armed (m3 motors) motor_index 3 motor_percent
  let m4_val = motorfun armed (m4 motors) motor_index 4 motor_percent

  trigger "stream_writeBrushedMotors" true [
        arg m1_pin, arg m2_pin, arg m3_pin, arg m4_pin,
        arg m1_val, arg m2_val, arg m3_val, arg m4_val]


-- Compile the spec
main = reify spec >>= compile "copilot"
