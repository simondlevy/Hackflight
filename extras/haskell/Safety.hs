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

getSafety :: Stream Bool -> State -> Safety

getSafety starting vehicleState = Safety armed failsafe

  where

    -- Use receiver data to trap failsafe
    failsafe = receiverLostSignal || failsafe' where failsafe' = [False] ++ failsafe

    -- Arm after safety checks
    armed = if failsafe then false 
            else if receiverAux1 > 0 && receiverThrottleIsDown && safeToArm vehicleState then true 
            else if receiverAux1 < 0 then false
            else armed' where armed' = [False] ++ armed

