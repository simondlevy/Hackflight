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

  -- Run the full Hackflight algorithm
  let (msgbuff, motors, led) = hackflight receiver sensors pidfuns QuadXMW

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

  -- Send reply to GCS if indicated
  trigger "serialSend" (sending msgbuff) [ 
                                        arg $ hdr0 msgbuff
                                      , arg $ hdr1 msgbuff
                                      , arg $ hdr2 msgbuff
                                      , arg $ outsize msgbuff
                                      , arg $ msgtype msgbuff
                                      , arg $ crc msgbuff
                                      , arg $ paysize msgbuff
                                      , arg $ val00 msgbuff
                                      , arg $ val01 msgbuff
                                      , arg $ val02 msgbuff
                                      , arg $ val03 msgbuff
                                      , arg $ val04 msgbuff
                                      , arg $ val05 msgbuff
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
