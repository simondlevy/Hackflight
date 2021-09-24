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
import Safety

data FullStatus = FullStatus {  ledOn       :: Stream Bool
                              , motorsReady :: Stream Bool }

hackflightFull :: Receiver -> [Sensor] -> [PidController] -> Mixer
  -> FullStatus

hackflightFull receiver sensors pidControllers mixer
  = FullStatus ledOn motorsReady

  where

    -- Run core algorithm
    (mixerMotors, vehicleState, safety) =
      hackflight receiver sensors pidControllers mixer getSafetyReal

    -- This allows us to set the motors periodically
    motorsReady = ready 66

    -- Blink LED during first couple of seconds; keep it solid when armed
    ledOn = if micros < 2000000 then (mod (div micros 50000) 2 == 0)
            else armed safety
