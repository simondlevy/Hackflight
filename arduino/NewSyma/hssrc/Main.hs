{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding(not, (==), (<), (>))

-- Core
import HackflightFull
import Safety
import Serial
import State
import Time
import Demands
import Receiver
import Mixer
import Pin

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid
import YawPid
import LevelPid

spec = do

  let led = Pin 18

  let motor1 = Pin 13
  let motor2 = Pin 16
  let motor3 = Pin 3
  let motor4 = Pin 11

  let receiver = makeReceiver 4.0

  -- These sensors will be run right-to-left via composition
  let sensors = [gyrometer, quaternion]

  let ratePid = rateController 0.225    -- Kp
                               0.001875 -- Ki
                               0.375    -- Kd
                               0.4      -- windupMax
                               40       -- maxDegreesPerSecond

  -- Set up some PID controllers

  let yawPid = yawController 2.0 -- Kp
                             0.1 -- Ki
                             0.4 -- windupMax

  let levelPid = levelController 0.2 -- Kp
                                 45  -- maxAngleDegrees

  -- Pos-hold goes first so that it can access roll/pitch demands from receiver
  let pidControllers = [ratePid, yawPid, levelPid]

  let mixer = quadXAPMixer

  -- Run the main Hackflight algorithm, getting the motor spins and LED state
  let (motors, ledState, serialBuffer, starting) = hackflightFull receiver
                                                                  sensors
                                                                  pidControllers
                                                                  mixer
  let looping = not starting

  -- Do some setup the first time around
  trigger "copilot_startSerial" starting []     
  trigger "copilot_startLed" starting [arg $ pin led]     
  trigger "copilot_startWire" starting []
  trigger "copilot_startUsfs" starting [] 
  trigger "copilot_startDsmrx" starting [] 
  trigger "copilot_startBrushedMotors" starting [  arg $ pin motor1
                                                 , arg $ pin motor2
                                                 , arg $ pin motor3
                                                 , arg $ pin motor4 ]
  -- Send the LED using external C function during the looping phase
  trigger "copilot_setLed" looping [arg $ pin led, arg ledState]

  -- Update serial comms and clock during the looping phase
  trigger "copilot_updateDsmrx" looping [] 
  trigger "copilot_updateUsfs" looping [] 
  trigger "copilot_updateSerial" looping [] 
  trigger "copilot_updateClock" looping [] 

  let serialCount = count serialBuffer
  let serialInBytes = inbytes serialBuffer
  let serialOutVals = outvals serialBuffer

  trigger "copilot_handleSerialInput" (inputReady serialBuffer)
                                      [  arg $ b00 serialInBytes
                                       , arg $ b01 serialInBytes 
                                       , arg $ b02 serialInBytes 
                                       , arg $ b03 serialInBytes 
                                       ] 

  trigger "copilot_sendSerialOutput" (serialCount > 0)
                                     [ arg $ msgtype serialBuffer
                                     , arg $ serialCount
                                     , arg $ v00 serialOutVals
                                     , arg $ v01 serialOutVals
                                     , arg $ v02 serialOutVals
                                     , arg $ v03 serialOutVals
                                     , arg $ v04 serialOutVals
                                     , arg $ v05 serialOutVals
                                     , arg $ v06 serialOutVals
                                     , arg $ v07 serialOutVals
                                     , arg $ v08 serialOutVals
                                     , arg $ v09 serialOutVals
                                     , arg $ v10 serialOutVals
                                     , arg $ v11 serialOutVals
                                     ]

  -- Send the motor values using the external C function
  trigger "copilot_writeBrushedMotors" looping [  arg $ pin motor1, arg $ (m1 motors),
                                                  arg $ pin motor2, arg $ (m2 motors),
                                                  arg $ pin motor3, arg $ (m3 motors),
                                                  arg $ pin motor4, arg $ (m4 motors) ]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
