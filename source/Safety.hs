{--
  Hackflight safety checking

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Safety where

import Language.Copilot

import Receiver
import State

data Safety = Safety { armed :: Stream Bool, failsafe :: Stream Bool }

type SafetyFun = State -> Safety

getSafetySim :: SafetyFun

-- Simulation is always armed and never loses signal
getSafetySim _ = Safety true false

getSafetyReal :: SafetyFun

getSafetyReal vehicleState = Safety armed failsafe

  where

    -- Use receiver data to trap failsafe
    failsafe = (armed' && receiverLostSignal) || failsafe' where 

    -- Arm after safety checks
    armed = if failsafe then false 

            else if receiverAux1 > 0
                 && receiverThrottleIsDown
                 && safeToArm vehicleState then true 

            else false

    armed' = [False] ++ armed
    failsafe' = [False] ++ failsafe
