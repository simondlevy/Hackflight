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

safety demands state = (armed, failsafe, mrunning, mzero)

  where

    -- Trip failsafe if armed and receiver lost signal
    failsafe = if failsafe' then true else armed' && receiverLostSignal

    -- Set motors to zero if disarmed or failsafe tripped or if armed with throttle down
    mzero = not armed' || failsafe || (armed' && throttleIsDown)

    -- Motors are ready to run if they are zeroed-out or if throttle is up in armed state
    mrunning = mzero || (armed' && not throttleIsDown)

    armed = if failsafe' then false  -- never armed once failsafe trips

            else if armed' && not aux1IsDown then false -- disarm if armed and aux1 is up

            -- arm after lots of safety checks
            else if (not armed' &&
                     not failsafe' &&
                     safeToArm state &&
                     throttleIsDown &&
                     aux1IsDown) then true

            else armed'

    throttleIsDown = (throttle demands) < (-0.995)

    aux1IsDown = receiverAux1 > 0

    armed' = [False] ++ armed
    failsafe' = [False] ++ failsafe
