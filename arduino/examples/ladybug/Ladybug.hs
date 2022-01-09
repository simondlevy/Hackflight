{--
  Support for Ladybug flight controller with DSMX receiver

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Ladybug where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import HackflightFull
import Receiver
import Demands
import State
import Mixers
import Motors
import MSP
import Messages
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

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Run the full Hackflight algorithm.  The "sending" flag will be true when
  -- the algorithm is ready to send data.
  let (message, sending, motors, led) = hackflight receiver sensors pidfuns QuadXMW

  -- Do some stuff at startup
  trigger "serialStart" starting []
  trigger "i2cStart" starting []
  trigger "dsmrxStart" starting []
  trigger "usfsStart" starting []
  trigger "brushedMotorsStart" starting [arg m1_pin, arg m2_pin, arg m3_pin, arg m4_pin]
  trigger "ledStart" starting [arg led_pin]

  -- Update receiver in loop
  trigger "dsmrxUpdate" running []
  trigger "dsmrxGet" receiverGotNewFrame []

  -- Do some other stuff in loop
  trigger "usfsUpdate" running []
  trigger "timeUpdate" running []
  trigger "ledWrite" running [arg led_pin, arg led]
  trigger "serialUpdate" running []
  trigger "serialRead" serialAvailable []

  -- Send reply to GCS if indicated.  C function serialSend() computes
  -- remaining CRC from floats before sending.
  trigger "serialSend" sending [ 
                                 arg $ hdr0 message
                               , arg $ hdr1 message
                               , arg $ hdr2 message
                               , arg $ outsize message
                               , arg $ msgtype message
                               , arg $ crc message
                               , arg $ paysize message
                               , arg $ val00 message
                               , arg $ val01 message
                               , arg $ val02 message
                               , arg $ val03 message
                               , arg $ val04 message
                               , arg $ val05 message
                               ]

  -- Run motors
  trigger "brushedMotorsWrite" true [
                                      arg m1_pin
                                    , arg m2_pin
                                    , arg m3_pin
                                    , arg m4_pin
                                    , arg $ m1 motors
                                    , arg $ m2 motors
                                    , arg $ m3 motors
                                    , arg $ m4 motors
                                    ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
