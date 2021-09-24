{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

-- Language
import Language.Copilot
import Copilot.Compile.C99

-- Core
import HackflightFull
import Receiver
import Time
import Mixer

-- Sensors
import Gyrometer
import Quaternion

-- PID controllers
import RatePid
import YawPid
import LevelPid

receiver = makeReceiver 4.0 -- demand scale

-- These sensors will be run right-to-left via composition
sensors = [gyrometer, quaternion]

  -- Set up some PID controllers --------------------------

                      -- Kp      Ki       Kd     windupMax maxDegreesPerSecond
ratePid = rateController 0.225   0.001875 0.375  0.4       40       

                    -- Kp  Ki  windupMax
yawPid = yawController 2.0 0.1 0.4 

                        -- Kp  maxAngleDegrees
levelPid = levelController 0.2 45 

pidControllers = [ratePid, yawPid, levelPid] 

mixer = quadXAPMixer

spec = do
  
  -- Run full Hackflight algorithm
  let status = hackflightFull receiver sensors pidControllers mixer

  -- Set LED
  trigger "copilot_setLed" true [arg $ ledOn status]

  -- Run motors
  let ms = motors status
  trigger "copilot_writeBrushedMotors" (motorsReady status) [  arg $ m1 ms
                                                             , arg $ m2 ms
                                                             , arg $ m3 ms
                                                             , arg $ m4 ms ]

-- Compile the spec
main = reify spec >>= compile "copilot"
