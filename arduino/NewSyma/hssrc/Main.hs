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
  let serialIn = input serialBuffer
  let serialOut = output serialBuffer

  trigger "copilot_handleSerialInput" (inputReady serialBuffer)
                                      [  arg $ w00 serialIn
                                       , arg $ w01 serialIn 
                                       , arg $ w02 serialIn 
                                       , arg $ w03 serialIn 
                                       ] 

  trigger "copilot_sendSerialOutput" (serialCount > 0)
                                     [ arg $ msgtype serialBuffer
                                     , arg $ serialCount
                                     , arg $ v00 serialOut
                                     , arg $ v01 serialOut
                                     , arg $ v02 serialOut
                                     , arg $ v03 serialOut
                                     , arg $ v04 serialOut
                                     , arg $ v05 serialOut
                                     , arg $ v06 serialOut
                                     , arg $ v07 serialOut
                                     , arg $ v08 serialOut
                                     , arg $ v09 serialOut
                                     , arg $ v10 serialOut
                                     , arg $ v11 serialOut
                                     ]

  -- Send the motor values using the external C function
  trigger "copilot_writeBrushedMotors"  looping  [  arg $ pin motor1, arg $ (m1 motors)
                                                  , arg $ pin motor2, arg $ (m2 motors)
                                                  , arg $ pin motor3, arg $ (m3 motors)
                                                  , arg $ pin motor4, arg $ (m4 motors) ]

  let ready = true

  trigger "copilot_debug" true [arg $ ready]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
