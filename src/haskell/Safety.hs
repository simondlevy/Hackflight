{--
  Hackflight safety checking

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Safety where

import Language.Copilot

import Receiver
import Demands
import State
import Utils

safety :: Demands -> State -> (SBool, SBool, SBool, SBool)

safety demands state = (armed, failsafe, mready, cut)

  where

    failsafe = if failsafe' then true else armed' && receiverLostSignal

    disarm = armed' && not aux1IsDown

    armedThrottleDown = armed' && throttleIsDown

    -- Arm after lots of safety checks
    arm = not armed' && not failsafe' && safeToArm state && throttleIsDown && aux1IsDown

    cut = failsafe || armedThrottleDown

    mready = cut || arm || (armed' && not throttleIsDown)

    armed = if failsafe' || disarm then false else if arm then true else armed'

    throttleIsDown = (throttle demands) < (-0.995)
    aux1IsDown = receiverAux1 > 0

    armed' = [False] ++ armed
    failsafe' = [False] ++ failsafe
