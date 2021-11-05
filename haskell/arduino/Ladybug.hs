{--
  Support for Ladybug flight controller quadcopter with DSMX receiver

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Ladybug where

import Language.Copilot hiding(xor)
import Copilot.Compile.C99
import Prelude hiding((++), (==), (&&), (/), (*), xor)

-- Core
import Hackflight
import Receiver
import Demands
import State
import Mixers
import Time

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
import Messages

-- Misc
import Utils

------------------------------------------------------------

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

------------------------------------------------------------

motorfun :: SBool -> SFloat -> SWord8 -> SWord8 -> SWord8 -> SFloat
motorfun armed flying_value index target percent =
  if armed then flying_value
  else if index == target then (unsafeCast percent) / 100
  else 0

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Run the Hackflight algorithm
  let (vstate, armed, motors, led) = hackflightFull receiver sensors pidfuns quadxmw

  -- Run the serial comms parser
  let (msgtyp, sending, payindex, checked) = parse serialAvailable serialByte

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startI2C" starting []
  trigger "stream_startDsmrx" starting []
  trigger "stream_startUsfs" starting []
  trigger "stream_startBrushedMotors" starting [arg m1_pin, arg m2_pin, arg m3_pin, arg m4_pin]
  trigger "stream_startLed" starting [arg led_pin]

  -- Do some other stuff in loop
  trigger "stream_updateUsfs" running []
  trigger "stream_updateDsmrx" running []
  trigger "stream_updateTime" running []
  trigger "stream_writeLed" running [arg led_pin, arg led]

  trigger "stream_serialUpdate" running []
  trigger "stream_serialRead" serialAvailable []

  -- Serial comms ---------------------------------------------------------------------

  let msgbuffer = message msgtyp vstate

  trigger "stream_serialSend" sending [ 
                                        arg $ hdr0 msgbuffer
                                      , arg $ hdr1 msgbuffer
                                      , arg $ hdr2 msgbuffer
                                      , arg $ outsize msgbuffer
                                      , arg $ msgtype msgbuffer
                                      , arg $ crc msgbuffer
                                      , arg $ paysize msgbuffer
                                      , arg $ val00 msgbuffer
                                      , arg $ val01 msgbuffer
                                      , arg $ val02 msgbuffer
                                      , arg $ val03 msgbuffer
                                      , arg $ val04 msgbuffer
                                      , arg $ val05 msgbuffer
                                      ]

  let motor_index = if msgtyp == 215 && payindex == 1 then serialByte
                    else motor_index' where motor_index' = [0] ++ motor_index

  let motor_percent = if msgtyp == 215 && payindex == 2 then serialByte
                      else motor_percent' where motor_percent' = [0] ++ motor_percent

  let m1_val = motorfun armed (m1 motors) motor_index 1 motor_percent
  let m2_val = motorfun armed (m2 motors) motor_index 2 motor_percent
  let m3_val = motorfun armed (m3 motors) motor_index 3 motor_percent
  let m4_val = motorfun armed (m4 motors) motor_index 4 motor_percent

  trigger "stream_writeBrushedMotors" true [
        arg m1_pin, arg m2_pin, arg m3_pin, arg m4_pin,
        arg m1_val, arg m2_val, arg m3_val, arg m4_val]

-- Compile the spec
main = reify spec >>= compile "hackflight"
