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

safety :: Demands -> State -> (SBool, SBool, SBool)

safety demands state = (armed, failsafe, mzero)

  where

    -- Trip failsafe if armed and receiver timed out
    failsafe = if failsafe' then true else armed' && c_receiverReady && c_receiverTimedOut

    -- Set motors to zero if disarmed or failsafe tripped or if armed with throttle down
    mzero = not armed' || failsafe || (armed' && throttleIsDown)

    -- Arm after lots of safety checks
    armed = if failsafe' then false  -- never armed once failsafe trips

            else if armed' && not aux1IsDown then false -- disarm if armed and aux1 is up

            else if (not armed' &&
                     not failsafe' &&
                     safeToArm state &&
                     throttleIsDown &&
                     aux1IsDown) then true

            else armed'

    throttleIsDown = (throttle demands) < (-0.995)

    aux1IsDown = c_receiverAux1 > 1500

    armed' = [False] ++ armed
    failsafe' = [False] ++ failsafe
