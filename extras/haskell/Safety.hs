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

    -- Aux switch determines arming
    auxState = receiverAux1 > 0

    -- Aux switch must be off before we can arm
    auxWasOff = (not starting) && (not auxState) || auxWasOff'
                where auxWasOff' = [False] ++ auxWasOff

    -- Arm after safety checks
    armed = not failsafe && safeToArm vehicleState && auxState && auxWasOff
