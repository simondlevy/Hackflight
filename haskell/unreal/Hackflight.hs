{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Hackflight where

import Language.Copilot

import Prelude hiding((!!), (||), (++), (<), (>), (&&), (==), div, mod, not)

import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Utils

hackflightSim :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> Motors

hackflightSim receiver sensors pidfuns mixer = motors

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the current state
    -- vstate = compose sensors (state' vstate)
    vstate = compose sensors (State 0 0 0 0 0 0 0 0 0 0 0 0)

    -- Get the demands by composing the PID control functions over the vehicle state and
    -- receiver demands.
    (_, pdemands) = compose pidfuns (vstate, rdemands)

    -- Apply mixer to demands to get motor values, returning motor values and LED state
    motors = mixer (\m -> constrain m) pdemands
