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
import Serial
import Utils

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid(rateController)
import YawPid(yawController)
import LevelPid(levelController)

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

  -- Debugging
  -- trigger "serial2Start" starting []

  -- Update receiver in loop
  trigger "dsmrxUpdate" running []
  trigger "dsmrxGet" c_receiverGotNewFrame []
  trigger "dsmrxGetInt" c_receiverGotNewFrame []

  -- Do some other stuff in loop
  trigger "usfsUpdate" running []
  trigger "timeUpdate" running []
  trigger "ledWrite" running [arg led_pin, arg led]
  trigger "serialUpdate" running []
  trigger "serialRead" c_serialAvailable []

  -- Sepcial handing for attitude messages
  let isatt = (msgtype message) == 108 :: SBool
  trigger "commsSendAttitude" (sending && isatt)
                             [  arg $ f1 message
                              , arg $ f2 message
                              , arg $ f3 message
                             ]

  trigger "commsSend" (sending && not isatt) [ 
                                arg $ direction message
                              , arg $ paysize message
                              , arg $ msgtype message
                              , arg $ f1 message
                              , arg $ f2 message
                              , arg $ f3 message
                              , arg $ f4 message
                              , arg $ f5 message
                              , arg $ f6 message
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

  -- trigger "serial2Debug" running [arg m1val, arg m2val, arg m3val, arg m4val]


-- Compile the spec
main = reify spec >>= compile "hackflight"
