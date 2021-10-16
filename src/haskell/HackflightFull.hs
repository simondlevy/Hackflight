{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Prelude hiding((!!), (||), (++), (<), (>), (&&), (==), div, mod, not)

import Receiver
import State
import Sensor
import Demands
import PidController
import Safety
import Utils

hackflight :: Receiver -> [Sensor] -> [PidFun] -> (State, Demands)

hackflight receiver sensors pidfuns = (state, demands)

  where

    -- Get receiver demands from external C functions
    demands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the initial state
    state = compose sensors state'

    -- Check safety (arming / failsafe)
    (armed, failsafe, mready, cut) = safety demands state

    -- Blink LED during first couple of seconds; keep it solid when armed
    -- ledOn = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else isArmed

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
