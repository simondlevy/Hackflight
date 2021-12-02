{--
  Support for TinyPico flight controller with DSMX receiver

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module TinyPico where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import HackflightFull
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
  trigger "stream_startSerial" starting []
  trigger "stream_startI2C" starting []
  trigger "stream_startDsmrx" starting []
  trigger "stream_startUsfs" starting []

  -- Do some other stuff in loop
  trigger "stream_updateUsfs" running []
  trigger "stream_updateDsmrx" running []
  trigger "stream_updateTime" running []
  trigger "stream_writeLed" running []
  trigger "stream_serialUpdate" running []
  trigger "stream_serialRead" stream_serialAvailable []

  -- Send message to GCS if indicated
  trigger "stream_serialSend" (sending msgbuff) [ 
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

-- Compile the spec
main = reify spec >>= compile "hackflight"
