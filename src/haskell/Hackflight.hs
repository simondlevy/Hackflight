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
import Demands
import Utils

hackflight :: Receiver -> [Sensor] -> (State, Demands)

hackflight receiver sensors = (state, demands)

  where

    -- Get receiver demands from external C functions
    demands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the initial state
    state = compose sensors zeroState
