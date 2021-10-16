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

    throttleIsDown = (throttle demands) < (-0.995)

    aux1IsDown = receiverAux1 > 0

    disarm = armed' && not aux1IsDown

    armedThrottleDown = armed' && throttleIsDown

    running = armed' && not throttleIsDown

    -- Arm after lots of safety checks
    arm = not armed' && not failsafe' && safeToArm state && throttleIsDown && aux1IsDown

    cut = failsafe || disarm || armedThrottleDown

    mready = if failsafe || disarm || arm || armedThrottleDown || running then true else mready'

    armed = if failsafe' || disarm then false else if arm then true else armed'

    failsafe = if failsafe' then true else failsafe'

    armed' = [False] ++ armed
    failsafe' = [False] ++ failsafe
    mready' = [False] ++ mready
