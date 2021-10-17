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
import Time
import Mixer
import Utils

hackflight :: Receiver -> [Sensor] -> [PidFun] -> (State, SBool, SBool, Demands, SBool)

hackflight receiver sensors pidfuns = (state, mready, mcut , pdemands, led)

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the current state
    state = compose sensors state'

    -- Periodically get the demands by composing the PID controllers over the receiver
    -- demands
    (_, _, pdemands) = compose pidfuns (state, timerReady 300, rdemands)

    -- Check safety (arming / failsafe)
    (armed, failsafe, mready, mcut) = safety rdemands state

    -- Blink LED during first couple of seconds; keep it solid when armed
    led = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else armed

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
