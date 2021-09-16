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

-- LED
import Led

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid
import YawPid
import LevelPid

spec = do

  let led = Led 18

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
  let (motors, ledState, serial, starting) = hackflight
                                             receiver
                                             sensors
                                             pidControllers
                                             mixer
                                             getSafetyReal
                                             getSerialOutReal

  let looping = not starting

  -- Do some setup the first time around
  trigger "copilot_startSerial" starting []     -- Serial comms
  trigger "copilot_startLed" starting [arg $ pin led]     -- LED
  trigger "copilot_startWire" starting []       -- I^2C
  trigger "copilot_startUsfs" starting []       -- IMU
  trigger "copilot_startDsmrx" starting []      -- Receiver
  
  -- Send the LED using external C function during the looping phase
  trigger "copilot_setLed" looping [arg $ pin led, arg ledState]

  -- Update serial comms and clock during the looping phase
  trigger "copilot_updateDsmrx" looping [] 
  trigger "copilot_updateUsfs" looping [] 
  trigger "copilot_updateSerial" looping [] 
  trigger "copilot_updateClock" looping [] 

  -- Send the motor values using the external C function
  trigger "copilot_writeMotor" looping [arg $ index (m1 motors), arg $ value (m1 motors)]
  trigger "copilot_writeMotor" looping [arg $ index (m2 motors), arg $ value (m2 motors)]
  trigger "copilot_writeMotor" looping [arg $ index (m3 motors), arg $ value (m3 motors)]
  trigger "copilot_writeMotor" looping [arg $ index (m4 motors), arg $ value (m4 motors)]

  -- Send and retrieve serial comms
  trigger "copilot_serialWrite" (available serial)  [arg (byte serial)]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
