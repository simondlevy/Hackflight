{--
  Hackflight algorithm for microcontrollers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Hackflight
import Time
import Receiver
import Sensor
import PidController
import Mixer
import Safety
import Demands
import Utils

hackflightFull :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> (Motors, SBool)

hackflightFull receiver sensors pidfuns mixer = (motors, ledOn)

  where

    -- Run core algorithm
    (motors, isArmed) = hackflight receiver sensors pidfuns mixer getSafetyReal

    -- Blink LED during first couple of seconds; keep it solid when armed
    ledOn = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else isArmed
