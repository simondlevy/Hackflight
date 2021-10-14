{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Hackflight where

import Language.Copilot

import Prelude hiding((!!), (||), (++), (<), (>), (&&), (==), div, mod, not)

import Safety
import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Utils

hackflight :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> SafetyFun -> (Motors, SBool)

hackflight receiver sensors pidfuns mixer safetyFun = (motors, isArmed)

  where

    -- Get receiver demands from external C functions
    receiverDemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the initial state
    state = compose sensors zeroState

    -- Get the demands by composing the PID control functions over the vehicle state and
    -- receiver demands.
    (_, demands) = compose pidfuns (state, receiverDemands)

    -- Get safety status
    safety = safetyFun state

    -- Apply mixer to demands to get motor values, returning motor values and LED state
    motors = mixer safety demands

    isArmed = armed safety
