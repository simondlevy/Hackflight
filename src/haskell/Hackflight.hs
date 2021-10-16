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
    state = compose sensors state'

    state' = State ([0] ++ (x state))
                   ([0] ++ (dx state))
                   ([0] ++ (y state))
                   ([0] ++ (dy state))
                   ([0] ++ (z state))
                   ([0] ++ (dz state))
                   ([0] ++ (phi state))
                   ([0] ++ (dphi state))
                   ([0] ++ (theta state))
                   ([0] ++ (dtheta state))
                   ([0] ++ (psi state))
                   ([0] ++ (dpsi state))
