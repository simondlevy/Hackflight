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
import Utils

data Safety = Safety { armed :: SBool, failsafe :: SBool }

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

            else if armed' && receiverAux1 < 0 then false
            
            else armed'

    armed' = [False] ++ armed
    failsafe' = [False] ++ failsafe
