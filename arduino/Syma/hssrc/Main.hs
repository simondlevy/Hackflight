{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding(not)

-- Core
import Hackflight
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
  let (motors, ledState, serialBuffer, starting) = hackflight
                                                   receiver
                                                   sensors
                                                   pidControllers
                                                   mixer
                                                   getSafetyReal
                                                   parseReal

  let looping = not starting

  -- Do some setup the first time around
  trigger "copilot_startSerial" starting []     
  trigger "copilot_startLed" starting [arg $ pin led]     
  trigger "copilot_startBrushedMotor" starting [arg $ pin led]
  trigger "copilot_startWire" starting []
  trigger "copilot_startUsfs" starting [] 
  trigger "copilot_startDsmrx" starting [] 
  trigger "copilot_startBrushedMotor" starting [arg $ pin motor1]
  trigger "copilot_startBrushedMotor" starting [arg $ pin motor2]
  trigger "copilot_startBrushedMotor" starting [arg $ pin motor3]
  trigger "copilot_startBrushedMotor" starting [arg $ pin motor4]
  
  -- Send the LED using external C function during the looping phase
  trigger "copilot_setLed" looping [arg $ pin led, arg ledState]

  -- Update serial comms and clock during the looping phase
  trigger "copilot_updateDsmrx" looping [] 
  trigger "copilot_updateUsfs" looping [] 
  trigger "copilot_updateSerial" looping [] 
  trigger "copilot_updateClock" looping [] 

  -- Send the motor values using the external C function
  trigger "copilot_writeBrushedMotor" looping [arg $ pin motor1, arg $ value (m1 motors)]
  trigger "copilot_writeBrushedMotor" looping [arg $ pin motor2, arg $ value (m2 motors)]
  trigger "copilot_writeBrushedMotor" looping [arg $ pin motor3, arg $ value (m3 motors)]
  trigger "copilot_writeBrushedMotor" looping [arg $ pin motor4, arg $ value (m4 motors)]

  -- Send and retrieve serial comms
 
-- Compile the spec
main = reify spec >>= compile "copilot"
