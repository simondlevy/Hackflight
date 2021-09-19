{--
  Hackflight algorithm with serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Prelude hiding((||), (++), (<), (>), (&&), (==), div, mod, not)

import Hackflight
import Safety
import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Time(time_msec)
import Serial
import Utils(compose)

hackflightFull :: Receiver -> [Sensor] -> [PidController] -> Mixer ->
    (Motors, Stream Bool, SerialBuffer, Stream Bool)

hackflightFull receiver sensors pidControllers mixer
  = (motors, led, serialBuffer, starting)

  where

    -- Run core algorithm
    (motors, vehicleState, safety) = hackflight receiver sensors pidControllers mixer getSafetyReal

    -- Blink LED on startup, keep solid when armed
    led = if time_msec < 5000 then (mod (div time_msec 50) 2 == 0) else armed safety

    -- Run serial comms
    (serialBuffer, serialMotors) = parse mixer vehicleState

    -- Track whether we've just started
    starting = [False] ++ true
