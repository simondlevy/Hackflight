{--
  Hackflight algorithm for microcontrollers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Hackflight
import Receiver
import Sensor
import PidController
import Mixer
import Time

data FullStatus = FullStatus {  starting    :: Stream Bool
                              , looping     :: Stream Bool
                              , ledOn       :: Stream Bool
                              , motorsReady :: Stream Bool }

hackflightFull :: Receiver -> [Sensor] -> [PidController] -> Mixer
  -> FullStatus

hackflightFull receiver sensors pidControllers mixer
  = FullStatus starting looping ledOn motorsReady

  where

    -- Run core algorithm
    -- (mixerMotors, vehicleState, safety) =
    --   hackflight receiver sensors pidControllers mixer getSafetyReal

    -- The looping flag will only be false on startup, so we
    -- can use it for initializtion in our main
    looping = not looping' where looping' = [False] ++ looping
    starting = not looping

    -- This allows us to set the motors periodically
    motorsReady = ready 66

    -- Blink LED during first couple of seconds; keep it solid when armed
    ledOn = if micros < 100000 then false 
            else if micros < 2000000 then (mod (div micros 50000) 2 == 0)
            else false -- armed
