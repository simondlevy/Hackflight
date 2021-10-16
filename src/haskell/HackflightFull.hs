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
import Utils

hackflight :: Receiver -> [Sensor] -> [PidFun] -> (State, Demands, SBool, Demands, SBool)

hackflight receiver sensors pidfuns = (state, rdemands, pready, pdemands, led)

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the initial state
    state = compose sensors state'

    -- Periodically update PID controls to modify demands
    pready = timerReady 300
    (_, _, pdemands) = compose pidfuns (state, pready, rdemands)

    -- Check safety (arming / failsafe)
    (armed, failsafe, mready, cut) = safety rdemands state

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
